"""
Câmera de profundidade virtual: tira uma "foto" da cena do PyBullet.

Rodar (de dentro da pasta nuvem_cilindro, com o venv ativo):
    python src/camera.py

Monta a cena do scenes/tubos.py sem abrir janela, captura uma vista e
mostra 3 imagens: RGB, profundidade (em metros) e máscara de segmentação.
A figura também é salva em data/camera_vista.png.
"""
import math
import sys
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pybullet as p

# Deixa o Python achar o scenes/tubos.py (fica numa pasta irmã de src/)
RAIZ = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(RAIZ / "scenes"))
import tubos  # noqa: E402

# ---------------------------------------------------------------------------
# 1. Parâmetros da câmera
#
#   largura, altura: resolução da imagem em pixels
#   fov:             abertura vertical da lente, em graus
#   near, far:       a câmera só enxerga entre essas distâncias (metros)
#   olho:            onde a câmera está no mundo
#   alvo:            para onde ela aponta
#   cima:            qual direção é "para cima" na foto (o Z do mundo)
# ---------------------------------------------------------------------------
LARGURA, ALTURA = 640, 480
FOV = 60.0
NEAR, FAR = 0.1, 10.0

OLHO = [2.6, -2.0, 1.9]
ALVO = [0.6, 0.0, 0.7]
CIMA = [0.0, 0.0, 1.0]


# ---------------------------------------------------------------------------
# 2. Converte o depth buffer do PyBullet para metros
#
# O getCameraImage não devolve a distância em metros, e sim um valor entre
# 0 e 1 que cresce de forma NÃO linear com a distância (é o "z-buffer" do
# OpenGL). Esta fórmula desfaz isso usando near e far.
# O resultado é a profundidade ao longo do eixo óptico da câmera (o "z" da
# câmera), não a distância em linha reta até o ponto.
# ---------------------------------------------------------------------------
def depth_para_metros(depth_buffer, near, far):
    return far * near / (far - (far - near) * depth_buffer)


# ---------------------------------------------------------------------------
# 3. Intrínsecos da câmera (o modelo "pinhole")
#
#   f:      distância focal em pixels, sai do FOV
#   cx, cy: centro óptico, o meio da imagem
# Com eles, um pixel (u, v) com profundidade z vira um ponto 3D:
#   x = (u - cx) * z / f      y = (v - cy) * z / f
# Isso é o passo 4 do roteiro (a nuvem de pontos).
# ---------------------------------------------------------------------------
def intrinsecos(largura, altura, fov_graus):
    f = altura / (2.0 * math.tan(math.radians(fov_graus) / 2.0))
    return {"f": f, "cx": largura / 2.0, "cy": altura / 2.0}


# ---------------------------------------------------------------------------
# 4. Tira a foto
# ---------------------------------------------------------------------------
def capturar(olho, alvo, cima=CIMA, largura=LARGURA, altura=ALTURA,
             fov=FOV, near=NEAR, far=FAR):
    # Matriz de vista: posição e direção da câmera no mundo
    view = p.computeViewMatrix(cameraEyePosition=olho,
                               cameraTargetPosition=alvo,
                               cameraUpVector=cima)
    # Matriz de projeção: a "lente"
    proj = p.computeProjectionMatrixFOV(fov=fov, aspect=largura / altura,
                                        nearVal=near, farVal=far)

    # ER_TINY_RENDERER é o renderizador por software: dá o mesmo resultado
    # com ou sem janela (GUI ou DIRECT)
    _, _, rgb, depth, mascara = p.getCameraImage(
        largura, altura, viewMatrix=view, projectionMatrix=proj,
        renderer=p.ER_TINY_RENDERER)

    # Dependendo de como o pybullet foi compilado, isso vem como lista ou
    # array; np.reshape deixa tudo como matriz altura x largura
    rgb = np.reshape(rgb, (altura, largura, 4))[:, :, :3].astype(np.uint8)
    depth = np.reshape(depth, (altura, largura))
    mascara = np.reshape(mascara, (altura, largura))

    return {
        "rgb": rgb,
        "profundidade": depth_para_metros(depth, near, far),  # metros
        "mascara": mascara,   # em cada pixel: id_pybullet do objeto (-1 = fundo)
        "view": view,         # vai servir para levar os pontos ao mundo
        "intrinsecos": intrinsecos(largura, altura, fov),
        "near": near, "far": far,
    }


# ---------------------------------------------------------------------------
# 5. Teste: captura uma vista, imprime um resumo e mostra as imagens
# ---------------------------------------------------------------------------
if __name__ == "__main__":
    p.connect(p.DIRECT)          # sem janela: só precisamos das imagens
    gt = tubos.montar_cena()
    foto = capturar(OLHO, ALVO)

    prof, masc = foto["profundidade"], foto["mascara"]
    print(f"Intrínsecos: {foto['intrinsecos']}")

    nomes = {gt["id_piso"]: "piso", -1: "fundo (céu)"}
    nomes.update({c["id_pybullet"]: c["nome"] for c in gt["cilindros"]})
    print("\nPixels por objeto e profundidade (m):")
    for id_obj, nome in nomes.items():
        sel = masc == id_obj
        if sel.sum() == 0:
            print(f"  {nome:14s} NÃO aparece na imagem!")
        elif id_obj == -1:
            print(f"  {nome:14s} {sel.sum():7d} px")
        else:
            print(f"  {nome:14s} {sel.sum():7d} px   "
                  f"de {prof[sel].min():.2f} a {prof[sel].max():.2f}")

    # Figura com as 3 imagens lado a lado
    fig, ax = plt.subplots(1, 3, figsize=(15, 4.5))
    ax[0].imshow(foto["rgb"])
    ax[0].set_title("RGB")

    # No fundo (céu) a profundidade bate no "far"; escondemos para a escala
    # de cor ficar boa nos objetos
    prof_vis = np.where(masc == -1, np.nan, prof)
    im = ax[1].imshow(prof_vis, cmap="viridis")
    ax[1].set_title("Profundidade (m)")
    fig.colorbar(im, ax=ax[1], fraction=0.035)

    ax[2].imshow(masc, cmap="tab10", interpolation="nearest")
    ax[2].set_title("Máscara de segmentação (id do objeto)")

    for a in ax:
        a.axis("off")
    fig.tight_layout()

    saida = RAIZ / "data" / "camera_vista.png"
    fig.savefig(saida, dpi=120)
    print(f"\nFigura salva em {saida}")
    plt.show()
