"""
Cena do "galho": um tubo comprido e inclinado saindo do chão, que a aranha
(o robô com a câmera) vai escalar.

Rodar (de dentro da pasta nuvem_cilindro, com o venv ativo):
    python scenes/galho.py

Abre a janela do PyBullet com a cena. Quem faz a câmera andar é o
src/aranha.py; este arquivo só monta o cenário e o ground truth.
"""
import math
import sys
import time
from pathlib import Path

import numpy as np
import pybullet as p
import pybullet_data

# Reaproveita a malha de 128 faces e o criar_cilindro do tubos.py
sys.path.insert(0, str(Path(__file__).resolve().parent))
import tubos  # noqa: E402

# ---------------------------------------------------------------------------
# 1. Parâmetros do galho (metros e graus)
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
# 2. Monta a cena: chão + galho. Devolve o ground truth no mesmo formato do
#    tubos.py, então o nuvem.conferir() funciona aqui também.
# ---------------------------------------------------------------------------
def montar_cena_galho(g=GALHO):
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)
    id_piso = p.loadURDF("plane.urdf")

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

    corpo, eixo = tubos.criar_cilindro(g["raio"], g["comprimento"],
                                       centro.tolist(), euler, g["cor"])

    return {
        "unidade": "m",
        "id_piso": id_piso,
        "cilindros": [{
            "nome": g["nome"],
            "id_pybullet": corpo,
            "raio": g["raio"],
            "altura": g["comprimento"],   # mesmo nome do tubos.py
            "centro": centro.tolist(),
            "eixo": eixo,
        }],
    }


if __name__ == "__main__":
    p.connect(p.GUI)
    gt = montar_cena_galho()
    c = gt["cilindros"][0]
    print(f"Galho: raio {c['raio']} m, comprimento {c['altura']} m, "
          f"eixo {np.round(c['eixo'], 3).tolist()}")
    # Posiciona a câmera da janela (só a visualização, não a da aranha)
    p.resetDebugVisualizerCamera(cameraDistance=5.0, cameraYaw=20,
                                 cameraPitch=-20,
                                 cameraTargetPosition=c["centro"])
    while p.isConnected():
        time.sleep(0.1)
