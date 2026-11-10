"""
Projeção: transforma a foto da câmera em pontos 3D no referencial do mundo.

Para gerar e ver a nuvem da cena dos tubos, rode scripts/gerar_nuvem.py.
"""
import numpy as np

from percepcao.nuvem import Nuvem
from percepcao.simulacao import camera

# ---------------------------------------------------------------------------
# 1. Configuração
#
#   PROF_MAX: descarta pontos mais longe que isso (metros). O piso vai até o
#             horizonte e geraria muitos pontos inúteis lá no fundo.
#   (As vistas da cena dos tubos ficam em cenas.VISTAS_TUBOS.)
# ---------------------------------------------------------------------------
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
# 5. Tira as fotos de uma lista de câmeras (olho, alvo, cima) e junta tudo
#    numa Nuvem só. Como tudo já está no referencial do mundo, juntar vistas
#    é só empilhar. Cada ponto guarda a posição da câmera que o viu
#    (origens).
# ---------------------------------------------------------------------------
def fotografar(vistas, near=camera.NEAR, prof_max=PROF_MAX):
    pts, cores, rots, origens = [], [], [], []
    for olho, alvo, cima in vistas:
        foto = camera.capturar(list(olho), list(alvo), list(cima), near=near)
        P, C, R = foto_para_nuvem(foto, prof_max=prof_max)
        pts.append(P)
        cores.append(C)
        rots.append(R)
        origens.append(np.tile(olho, (len(P), 1)))
    return Nuvem(pontos=np.vstack(pts), rotulos=np.concatenate(rots),
                 cores=np.vstack(cores), origens=np.vstack(origens))
