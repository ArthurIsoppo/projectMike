"""
A aranha: uma câmera que escala o galho, andando ao longo do eixo do tubo,
um pouco acima da superfície, e tirando uma foto a cada passo.

Rodar (de dentro da pasta nuvem_cilindro, com o venv ativo):
    python src/aranha.py              sem janela, só gera os quadros
    python src/aranha.py gui          abre a janela e mostra a aranha subindo

Gera em data/galho/:
    quadro_000.ply, quadro_001.ply, ...   nuvem de cada passo (referencial do MUNDO)
    quadro_000_rotulos.npy, ...           objeto de cada ponto (0 = piso, 1 = galho)
    gt.json                               ground truth do galho
    trajetoria.json                       pose da câmera em cada quadro
"""
import json
import math
import sys
import time
from pathlib import Path

import numpy as np
import open3d as o3d
import pybullet as p

RAIZ = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(RAIZ / "scenes"))
sys.path.insert(0, str(RAIZ / "src"))
import galho   # noqa: E402
import tubos   # noqa: E402
import camera  # noqa: E402
import nuvem   # noqa: E402

# ---------------------------------------------------------------------------
# 1. Parâmetros da aranha
#
#   ALTURA:   distância da câmera até a superfície do tubo (metros).
#             O plano pede para testar algumas alturas: é aqui que muda.
#   FRENTE:   quão à frente, ao longo do tubo, a câmera mira (metros).
#             Maior = olha mais "para a frente"; menor = olha mais para baixo.
#             Com FRENTE = ALTURA, a câmera olha 45° para baixo e vê só um
#             trecho curto do tubo (o caso difícil que o plano pede).
#   INICIO, FIM: trecho do tubo que a aranha percorre, medido a partir da base.
#   N_QUADROS:   quantas fotos tira no caminho.
#   PROF_MAX:    descarta pontos a mais de X metros da câmera.
#   NEAR:        a câmera está muito perto do tubo, então o "near" de 10 cm
#                do camera.py cortaria a superfície. Usamos 2 cm.
# ---------------------------------------------------------------------------
ALTURA = 0.15
FRENTE = 0.15
INICIO, FIM = 0.3, 5.0
N_QUADROS = 20
PROF_MAX = 3.0
NEAR = 0.02

PASTA = RAIZ / "data" / "galho"


# ---------------------------------------------------------------------------
# 2. Onde a aranha está e para onde olha, a uma distância s da base
#
# Três vetores definem a pose:
#   eixo: direção do tubo (para onde a aranha anda)
#   cima: o "para cima" da aranha. É perpendicular ao eixo e aponta para o
#         lado de cima do tubo. Sai do Z do mundo, tirando a parte que está
#         na direção do eixo (a mesma conta da "distância ao eixo").
#   olho: ponto no eixo + (raio + ALTURA) na direção "cima"
#   alvo: um ponto da superfície, FRENTE metros adiante
#
# Por que não usar o Z do mundo como "cima" da câmera, como no camera.py?
# Se o tubo fosse vertical, a câmera olharia na mesma direção do Z, e o
# "cima" ficaria indefinido (a foto poderia sair girada de qualquer jeito).
# Usando o "cima" do próprio tubo, funciona em qualquer inclinação.
# ---------------------------------------------------------------------------
def pose_aranha(cil, s, altura=ALTURA, frente=FRENTE):
    eixo = np.array(cil["eixo"])
    base = np.array(cil["centro"]) - eixo * cil["altura"] / 2

    z_mundo = np.array([0.0, 0.0, 1.0])
    if abs(z_mundo @ eixo) > 0.99:            # tubo quase vertical
        z_mundo = np.array([1.0, 0.0, 0.0])
    cima = z_mundo - (z_mundo @ eixo) * eixo
    cima /= np.linalg.norm(cima)

    olho = base + s * eixo + (cil["raio"] + altura) * cima
    alvo = base + (s + frente) * eixo + cil["raio"] * cima
    return olho, alvo, cima


# ---------------------------------------------------------------------------
# 3. O que a aranha viu do galho em um quadro
#
#   arco:   quantos graus da volta do tubo aparecem. A aranha vê uma faixa
#           contínua em volta do topo, então basta o maior ângulo menos o
#           menor (o topo é 0°, um lado é negativo e o outro positivo)
#   trecho: de onde até onde, ao longo do tubo, aparecem pontos
#   teorico: o maior arco que dá para ver de um ponto a uma distância D do
#            eixo é 2 * arccos(raio / D). Com raio 0,10 e D = 0,25, dá 133°.
#            Serve para conferir se o arco medido faz sentido.
#   raio:   mediana da distância dos pontos ao eixo (deveria ser o raio)
# ---------------------------------------------------------------------------
def medir_quadro(pontos, rotulos, cil, cima):
    eixo = np.array(cil["eixo"])
    base = np.array(cil["centro"]) - eixo * cil["altura"] / 2
    sel = pontos[rotulos == cil["id_pybullet"]]
    if len(sel) == 0:
        return None

    rel = sel - base
    t = rel @ eixo                         # posição ao longo do tubo
    radial = rel - np.outer(t, eixo)       # parte perpendicular ao eixo
    dist = np.linalg.norm(radial, axis=1)

    # Ângulo de cada ponto em volta do eixo, com 0° = topo do tubo ("cima")
    lado = np.cross(eixo, cima)
    angulo = np.degrees(np.arctan2(radial @ lado, radial @ cima))

    return {
        "pontos_galho": int(len(sel)),
        "arco_graus": float(angulo.max() - angulo.min()),
        "trecho_m": [float(t.min()), float(t.max())],
        "raio_mediano_mm": float(np.median(dist) * 1000),
    }


def salvar_quadro(i, pontos, cores, rotulos):
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(pontos)
    pcd.colors = o3d.utility.Vector3dVector(cores)
    o3d.io.write_point_cloud(str(PASTA / f"quadro_{i:03d}.ply"), pcd)
    np.save(PASTA / f"quadro_{i:03d}_rotulos.npy", rotulos)


# ---------------------------------------------------------------------------
# 4. Desenho da aranha na janela (só no modo gui)
#
# Usamos linhas de debug, e não um objeto de verdade, porque linhas de debug
# não aparecem na foto da câmera. Um objeto no lugar da câmera taparia a
# visão dela.
# ---------------------------------------------------------------------------
def desenhar_aranha(olho, alvo, cima, ids):
    a = 0.08                                    # tamanho das "patas"
    linhas = [
        (olho, alvo, [1, 0.8, 0]),               # amarelo: para onde olha
        (olho, olho - cima * (ALTURA * 0.9), [0, 0, 0]),   # preto: até o tubo
        (olho - [a, 0, 0], olho + [a, 0, 0], [0, 0, 0]),
        (olho - [0, a, 0], olho + [0, a, 0], [0, 0, 0]),
    ]
    novos = []
    for k, (ini, fim, cor) in enumerate(linhas):
        kwargs = {"replaceItemUniqueId": ids[k]} if ids else {}
        novos.append(p.addUserDebugLine(list(ini), list(fim), cor,
                                        lineWidth=3, **kwargs))
    return novos


# ---------------------------------------------------------------------------
# 5. Junta tudo: a aranha sobe o galho, quadro a quadro
# ---------------------------------------------------------------------------
if __name__ == "__main__":
    com_janela = "gui" in sys.argv

    p.connect(p.GUI if com_janela else p.DIRECT)
    gt = galho.montar_cena_galho()
    cil = gt["cilindros"][0]

    PASTA.mkdir(parents=True, exist_ok=True)
    tubos.salvar_gt(gt, PASTA / "gt.json")

    D = cil["raio"] + ALTURA
    arco_teorico = 2 * math.degrees(math.acos(cil["raio"] / D))
    print(f"Altura da câmera: {ALTURA * 100:.0f} cm acima do tubo; "
          f"arco máximo visível (teórico): {arco_teorico:.0f}°\n")

    if com_janela:
        # Câmera da janela PARADA, vendo o galho inteiro de lado, para dar
        # para ver a aranha subindo (se ela seguisse a aranha, a aranha
        # ficaria sempre no meio da tela e pareceria parada)
        p.resetDebugVisualizerCamera(cameraDistance=5.5, cameraYaw=0,
                                     cameraPitch=-10,
                                     cameraTargetPosition=cil["centro"])

    posicoes = np.linspace(INICIO, FIM, N_QUADROS)
    trajetoria, ids_desenho = [], None
    for i, s in enumerate(posicoes):
        olho, alvo, cima = pose_aranha(cil, s)

        if com_janela:
            # Anda suave do quadro anterior até este, em 10 passinhos.
            # Só desenha: a foto é tirada uma vez, no fim do caminho.
            s_antes = posicoes[i - 1] if i > 0 else s
            for s_meio in np.linspace(s_antes, s, 10):
                o, a, c = pose_aranha(cil, s_meio)
                ids_desenho = desenhar_aranha(o, a, c, ids_desenho)
                time.sleep(0.03)
            # Rastro: um pontinho vermelho onde cada foto foi tirada
            p.addUserDebugPoints([olho.tolist()], [[1, 0, 0]], pointSize=6)

        foto = camera.capturar(olho.tolist(), alvo.tolist(), cima.tolist(),
                               near=NEAR)
        pontos, cores, rotulos = nuvem.foto_para_nuvem(foto, prof_max=PROF_MAX)
        salvar_quadro(i, pontos, cores, rotulos)

        m = medir_quadro(pontos, rotulos, cil, cima)
        trajetoria.append({"quadro": i, "s": float(s), "olho": olho.tolist(),
                           "alvo": alvo.tolist(), "cima": cima.tolist(),
                           "medidas": m})
        if m:
            print(f"quadro {i:2d}  s = {s:4.2f} m | {len(pontos):6d} pts "
                  f"(galho {m['pontos_galho']:6d}) | arco {m['arco_graus']:3.0f}° | "
                  f"trecho {m['trecho_m'][0]:4.2f}-{m['trecho_m'][1]:4.2f} m | "
                  f"raio {m['raio_mediano_mm']:5.1f} mm")
        else:
            print(f"quadro {i:2d}  s = {s:4.2f} m | o galho não aparece!")

    with open(PASTA / "trajetoria.json", "w") as f:
        json.dump(trajetoria, f, indent=2)
    print(f"\nSalvo em {PASTA}: {N_QUADROS} quadros, gt.json e trajetoria.json")

    if com_janela:
        print("Feche a janela para terminar.")
        while p.isConnected():
            time.sleep(0.1)
