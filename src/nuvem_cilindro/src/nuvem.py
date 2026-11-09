"""
Nuvem de pontos: transforma a foto da câmera em pontos 3D e salva em .ply.

Rodar (de dentro da pasta nuvem_cilindro, com o venv ativo):
    python src/nuvem.py       usa todas as vistas da lista VISTAS
    python src/nuvem.py 1     usa só a primeira vista (bom para comparar)

Gera em data/:
    nuvem.ply    pontos 3D (no referencial do MUNDO) com cor
    rotulos.npy  para cada ponto, o id_pybullet do objeto (0 = piso, 1-3 = cilindros)
    gt.json      ground truth dos cilindros (o mesmo do tubos.py)
"""
import sys
from pathlib import Path

import numpy as np
import open3d as o3d
import pybullet as p

RAIZ = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(RAIZ / "scenes"))
sys.path.insert(0, str(RAIZ / "src"))
import tubos   # noqa: E402
import camera  # noqa: E402

# ---------------------------------------------------------------------------
# 1. Configuração
#
#   VISTAS:   de onde a câmera fotografa, como pares (olho, alvo).
#             São 3 câmeras em volta da cena, separadas de 120° entre si,
#             todas a 1,9 m de altura e mirando o mesmo ponto (o meio da
#             cena). A primeira é a mesma do camera.py.
#   PROF_MAX: descarta pontos mais longe que isso (metros). O piso vai até o
#             horizonte e geraria muitos pontos inúteis lá no fundo.
# ---------------------------------------------------------------------------
VISTAS = [
    (camera.OLHO,          camera.ALVO),   # frente-direita
    ([1.33, 2.73, 1.9],    camera.ALVO),   # fundo
    ([-2.13, -0.73, 1.9],  camera.ALVO),   # esquerda
]
PROF_MAX = 5.0


# ---------------------------------------------------------------------------
# 2. Profundidade -> pontos 3D no referencial da CÂMERA
#
# Modelo pinhole: um pixel (u, v) com profundidade z vira
#     x = (u - cx) * z / f
#     y = (v - cy) * z / f
#
# Dois detalhes do PyBullet:
#   a) A câmera segue a convenção do OpenGL: X para a direita, Y para CIMA e
#      a câmera olha para -Z. Na imagem, v cresce para BAIXO, por isso o
#      sinal de menos no y, e o ponto fica em -z.
#   b) O TinyRenderer conta os pixels com um deslocamento de 1 na vertical
#      (v + 1). Sem isso o chão sai a até 2 cm de altura em vez de z = 0.
#      Descobrimos isso medindo: com v + 1 o erro do chão é 0,0 mm.
# ---------------------------------------------------------------------------
def pontos_camera(profundidade, intr):
    altura, largura = profundidade.shape
    # v = número da linha, u = número da coluna, para todos os pixels de uma vez
    v, u = np.mgrid[0:altura, 0:largura]
    z = profundidade
    x = (u - intr["cx"]) * z / intr["f"]
    y = -(v + 1 - intr["cy"]) * z / intr["f"]
    return np.stack([x, y, -z], axis=-1)   # formato: altura x largura x 3


# ---------------------------------------------------------------------------
# 3. Referencial da câmera -> referencial do MUNDO
#
# A view matrix leva pontos do mundo para a câmera. Para fazer o caminho
# contrário usamos a inversa dela.
# O PyBullet devolve a matriz como 16 números "por coluna", então o
# reshape(4, 4) precisa do .T (transposta) para ficar na forma normal.
# Coordenadas homogêneas: [x, y, z, 1] deixa a rotação e a translação numa
# única multiplicação de matriz.
# ---------------------------------------------------------------------------
def camera_para_mundo(pts_cam, view):
    V = np.array(view).reshape(4, 4).T
    V_inv = np.linalg.inv(V)
    n = pts_cam.shape[0]
    homog = np.hstack([pts_cam, np.ones((n, 1))])   # N x 4
    return (homog @ V_inv.T)[:, :3]


# ---------------------------------------------------------------------------
# 4. Uma foto -> pontos no mundo, cores e rótulos
# ---------------------------------------------------------------------------
def foto_para_nuvem(foto, prof_max=PROF_MAX):
    pts = pontos_camera(foto["profundidade"], foto["intrinsecos"])

    # Fica só com os pixels que acertaram algum objeto (máscara -1 = céu)
    # e que estão perto o suficiente
    manter = (foto["mascara"] >= 0) & (foto["profundidade"] < prof_max)

    pts_cam = pts[manter]                         # N x 3
    cores = foto["rgb"][manter] / 255.0           # Open3D usa cor de 0 a 1
    rotulos = foto["mascara"][manter]
    return camera_para_mundo(pts_cam, foto["view"]), cores, rotulos


# ---------------------------------------------------------------------------
# 5. Conferência: os pontos de cada cilindro estão mesmo a "raio" do eixo?
#
# Para cada ponto, calculamos a distância até a reta do eixo do GT.
# Se a conversão estiver certa, essa distância é igual ao raio.
# Os pontos das tampas (pontas do cilindro) ficam fora da conta, porque
# lá a distância ao eixo vai de 0 até o raio.
# ---------------------------------------------------------------------------
def conferir(pontos, rotulos, gt):
    piso = pontos[rotulos == gt["id_piso"]]
    print(f"  piso: {len(piso):6d} pontos, |z| máximo = "
          f"{np.abs(piso[:, 2]).max() * 1000:.2f} mm (deveria ser ~0)")

    for c in gt["cilindros"]:
        sel = pontos[rotulos == c["id_pybullet"]]
        if len(sel) == 0:
            print(f"  {c['nome']}: nenhum ponto!")
            continue
        eixo, centro = np.array(c["eixo"]), np.array(c["centro"])
        rel = sel - centro
        t = rel @ eixo                              # posição ao longo do eixo
        lateral = np.abs(t) < c["altura"] / 2 - 0.005   # tira as tampas
        dist = np.linalg.norm(rel - np.outer(t, eixo), axis=1)[lateral]
        erro = (dist - c["raio"]) * 1000

        # Cobertura: quantos graus da volta do cilindro têm pontos.
        # Montamos dois vetores u e w perpendiculares ao eixo (um "relógio"
        # em volta do eixo), medimos o ângulo de cada ponto nesse relógio e
        # contamos quantas fatias de 10° têm pelo menos um ponto.
        ref = [1.0, 0.0, 0.0] if abs(eixo[0]) < 0.9 else [0.0, 1.0, 0.0]
        u = np.cross(eixo, ref)
        u /= np.linalg.norm(u)
        w = np.cross(eixo, u)
        radial = (rel - np.outer(t, eixo))[lateral]
        angulo = np.degrees(np.arctan2(radial @ w, radial @ u))   # -180 a 180
        fatias = np.unique(((angulo + 180) // 10).astype(int))
        cobertura = 10 * len(fatias)

        print(f"  {c['nome']:13s} {len(sel):6d} pontos, raio GT "
              f"{c['raio'] * 1000:.0f} mm, medido {np.median(dist) * 1000:.1f} mm "
              f"(erro máx {np.abs(erro).max():.1f} mm), "
              f"cobertura {cobertura}° de 360°")


def salvar(pontos, cores, rotulos, pasta):
    nuvem = o3d.geometry.PointCloud()
    nuvem.points = o3d.utility.Vector3dVector(pontos)
    nuvem.colors = o3d.utility.Vector3dVector(cores)
    o3d.io.write_point_cloud(str(pasta / "nuvem.ply"), nuvem)
    np.save(pasta / "rotulos.npy", rotulos)
    return nuvem


# ---------------------------------------------------------------------------
# 6. Junta tudo
# ---------------------------------------------------------------------------
if __name__ == "__main__":
    p.connect(p.DIRECT)
    gt = tubos.montar_cena()
    pasta = RAIZ / "data"
    tubos.salvar_gt(gt, pasta / "gt.json")

    # Quantas vistas usar: o número passado no terminal, ou todas.
    # sys.argv é a lista do que foi digitado: ["src/nuvem.py", "1"]
    n_vistas = int(sys.argv[1]) if len(sys.argv) > 1 else len(VISTAS)
    vistas = VISTAS[:n_vistas]

    todos_pts, todas_cores, todos_rot = [], [], []
    for olho, alvo in vistas:
        foto = camera.capturar(olho, alvo)
        pts, cores, rot = foto_para_nuvem(foto)
        todos_pts.append(pts)
        todas_cores.append(cores)
        todos_rot.append(rot)
        print(f"Vista {olho}: {len(pts)} pontos")

    # Como tudo já está no referencial do mundo, juntar vistas é só empilhar
    pontos = np.vstack(todos_pts)
    cores = np.vstack(todas_cores)
    rotulos = np.concatenate(todos_rot)

    print("\nConferência contra o ground truth:")
    conferir(pontos, rotulos, gt)

    nuvem = salvar(pontos, cores, rotulos, pasta)
    print(f"\nSalvo: {pasta / 'nuvem.ply'} ({len(pontos)} pontos) e rotulos.npy")

    # Abre o visualizador do Open3D. Mouse: botão esquerdo gira, roda dá zoom,
    # Ctrl + esquerdo move. Os eixos coloridos têm 0,5 m (X vermelho,
    # Y verde, Z azul) e ficam na origem do mundo.
    eixos = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.5)
    o3d.visualization.draw_geometries([nuvem, eixos], window_name="Nuvem de pontos")
