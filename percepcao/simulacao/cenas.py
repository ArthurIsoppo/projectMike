"""
As cenas de teste e o ground truth (GT) delas.

  tubos: um piso e 3 cilindros com medidas conhecidas
  galho: um tubo comprido e inclinado saindo do chão, que a aranha (o robô
         com a câmera) vai escalar

As duas funções de montar devolvem o GT num formato só, o mesmo que a etapa
de primitivas vai devolver (ver percepcao/nuvem.py):

    {"unidade": "m", "primitivas": [
        {"tipo": "plano", "nome": "piso", "id_pybullet": 0,
         "normal": [0, 0, 1], "d": 0},
        {"tipo": "cilindro", "nome": ..., "id_pybullet": ..., "raio": ...,
         "altura": ..., "centro": [...], "eixo": [...]},
        ...
    ]}

Para abrir a janela com uma cena, rode scripts/ver_cena.py.
"""
import json
import math

import numpy as np
import pybullet as p
import pybullet_data

from percepcao.simulacao import camera
from percepcao.simulacao.malha import criar_cilindro

# ---------------------------------------------------------------------------
# 1a. Parâmetros dos cilindros da cena dos tubos (tudo em metros e radianos)
#
#   raio, altura: tamanho do cilindro
#   centro:       posição do CENTRO do cilindro no mundo (não da base)
#   euler:        rotação [em X, em Y, em Z]. [0, 0, 0] = cilindro em pé,
#                 com o eixo apontando para cima (eixo Z do mundo)
#   cor:          [vermelho, verde, azul, opacidade], de 0 a 1
# ---------------------------------------------------------------------------
CILINDROS = [
    {   # pilar grosso, em pé: centro em z = altura/2 para encostar no chão
        "nome": "pilar",
        "raio": 0.15, "altura": 2.0,
        "centro": [0.0, 0.0, 1.0],
        "euler": [0.0, 0.0, 0.0],
        "cor": [0.8, 0.2, 0.2, 1.0],
    },
    {   # tubo médio deitado no chão: girado 90° em Y, então o eixo fica
        # ao longo de X. Deitado, o centro fica em z = raio
        "nome": "tubo_piso",
        "raio": 0.08, "altura": 1.5,
        "centro": [1.0, 0.8, 0.08],
        "euler": [0.0, math.pi / 2, 0.0],
        "cor": [0.2, 0.6, 0.2, 1.0],
    },
    {   # tubo fino inclinado, suspenso no ar: 45° em X e 30° em Z
        "nome": "tubo_elevado",
        "raio": 0.05, "altura": 1.8,
        "centro": [0.8, -0.8, 1.2],
        "euler": [math.pi / 4, 0.0, math.pi / 6],
        "cor": [0.2, 0.3, 0.8, 1.0],
    },
]

# ---------------------------------------------------------------------------
# 1b. De onde a câmera fotografa a cena dos tubos, como (olho, alvo, cima).
#     São 3 câmeras em volta da cena, separadas de 120° entre si, todas a
#     1,9 m de altura e mirando o mesmo ponto (o meio da cena). A primeira é
#     a câmera padrão do camera.py.
# ---------------------------------------------------------------------------
VISTAS_TUBOS = [
    (camera.OLHO,          camera.ALVO, camera.CIMA),   # frente-direita
    ([1.33, 2.73, 1.9],    camera.ALVO, camera.CIMA),   # fundo
    ([-2.13, -0.73, 1.9],  camera.ALVO, camera.CIMA),   # esquerda
]

# ---------------------------------------------------------------------------
# 1c. Parâmetros do galho (metros e graus)
#
#   raio:        raio do tubo
#   comprimento: tamanho do tubo de ponta a ponta
#   inclinacao:  ângulo do tubo em relação ao chão (0 = deitado, 90 = em pé)
#   base:        onde fica o centro da ponta de baixo do tubo
# ---------------------------------------------------------------------------
GALHO = {
    "nome": "galho",
    "raio": 0.10,
    "comprimento": 6.0,
    "inclinacao": 35.0,
    "base": [0.0, 0.0, 0.10],
    "cor": [0.55, 0.35, 0.2, 1.0],
}


# ---------------------------------------------------------------------------
# 2. O chão, que as duas cenas têm. Entra no GT como plano: normal (0, 0, 1)
#    e d = 0, porque o chão é o conjunto de pontos com
#    0*x + 0*y + 1*z + 0 = 0, ou seja, z = 0.
# ---------------------------------------------------------------------------
def _criar_piso():
    p.setAdditionalSearchPath(pybullet_data.getDataPath())  # onde fica o plane.urdf
    p.setGravity(0, 0, -9.81)
    id_piso = p.loadURDF("plane.urdf")
    return {"tipo": "plano", "nome": "piso", "id_pybullet": id_piso,
            "normal": [0.0, 0.0, 1.0], "d": 0.0}


# ---------------------------------------------------------------------------
# 3. Monta a cena dos tubos e devolve o ground truth
# ---------------------------------------------------------------------------
def montar_tubos():
    gt = {"unidade": "m", "primitivas": [_criar_piso()]}

    for c in CILINDROS:
        corpo, eixo = criar_cilindro(c["raio"], c["altura"], c["centro"],
                                     c["euler"], c["cor"])
        gt["primitivas"].append({
            "tipo": "cilindro",
            "nome": c["nome"],
            "id_pybullet": corpo,   # vai servir para a máscara de segmentação
            "raio": c["raio"],
            "altura": c["altura"],
            "centro": c["centro"],
            "eixo": eixo,
        })

    return gt


# ---------------------------------------------------------------------------
# 4. Monta a cena do galho: chão + galho. Devolve o ground truth no mesmo
#    formato da cena dos tubos, então o avaliacao.conferir() funciona aqui
#    também.
# ---------------------------------------------------------------------------
def montar_galho(g=GALHO):
    piso = _criar_piso()

    # O cilindro nasce em pé (eixo = Z). Girar em Y por (90° - inclinação)
    # tomba o eixo na direção do X até ficar com a inclinação pedida:
    #   eixo = (cos(incl), 0, sen(incl))
    giro_y = math.radians(90.0 - g["inclinacao"])
    euler = [0.0, giro_y, 0.0]

    # O criar_cilindro pede o CENTRO; a gente sabe a BASE.
    # Centro = base + eixo * (comprimento / 2)
    incl = math.radians(g["inclinacao"])
    eixo_previsto = np.array([math.cos(incl), 0.0, math.sin(incl)])
    centro = np.array(g["base"]) + eixo_previsto * g["comprimento"] / 2

    corpo, eixo = criar_cilindro(g["raio"], g["comprimento"],
                                 centro.tolist(), euler, g["cor"])

    return {
        "unidade": "m",
        "primitivas": [piso, {
            "tipo": "cilindro",
            "nome": g["nome"],
            "id_pybullet": corpo,
            "raio": g["raio"],
            "altura": g["comprimento"],   # mesmo nome da cena dos tubos
            "centro": centro.tolist(),
            "eixo": eixo,
        }],
    }


def salvar_gt(gt, caminho):
    caminho.parent.mkdir(parents=True, exist_ok=True)
    with open(caminho, "w") as f:
        json.dump(gt, f, indent=2)
