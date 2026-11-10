"""
Câmera de profundidade virtual: tira uma "foto" da cena do PyBullet.

Para ver as 3 imagens (RGB, profundidade e máscara), rode
scripts/ver_camera.py.
"""
import math

import numpy as np
import pybullet as p

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
