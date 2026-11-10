"""
Abre a janela do PyBullet com uma cena.

Rodar (com o venv ativo):
    python scripts/ver_cena.py tubos     o piso e os 3 cilindros; salva o
                                         ground truth em data/gt.json
    python scripts/ver_cena.py galho     o chão e o galho que a aranha escala
                                         (quem faz a câmera andar é o
                                         scripts/aranha.py)

Sem argumento, abre a cena dos tubos.
"""
import sys
import time
from pathlib import Path

import numpy as np
import pybullet as p

from percepcao.simulacao import cenas

# Pasta data/ na raiz do projeto, funcione de onde o script for chamado
PASTA_DATA = Path(__file__).resolve().parent.parent / "data"


if __name__ == "__main__":
    cena = sys.argv[1] if len(sys.argv) > 1 else "tubos"
    if cena not in ("tubos", "galho"):
        sys.exit(f"Cena desconhecida: {cena!r}. Use tubos ou galho.")

    p.connect(p.GUI)

    if cena == "tubos":
        gt = cenas.montar_tubos()
        cenas.salvar_gt(gt, PASTA_DATA / "gt.json")
        print(f"Ground truth salvo em {PASTA_DATA / 'gt.json'}")
    else:
        gt = cenas.montar_galho()
        c = [pr for pr in gt["primitivas"] if pr["tipo"] == "cilindro"][0]
        print(f"Galho: raio {c['raio']} m, comprimento {c['altura']} m, "
              f"eixo {np.round(c['eixo'], 3).tolist()}")
        # Posiciona a câmera da janela (só a visualização, não a da aranha)
        p.resetDebugVisualizerCamera(cameraDistance=5.0, cameraYaw=20,
                                     cameraPitch=-20,
                                     cameraTargetPosition=c["centro"])

    # Mantém a janela aberta até você fechar.
    # Como os cilindros são estáticos, não precisa simular a física.
    while p.isConnected():
        time.sleep(0.1)
