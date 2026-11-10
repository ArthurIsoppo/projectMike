"""
Testa a câmera de profundidade virtual.

Rodar (com o venv ativo):
    python scripts/ver_camera.py

Monta a cena dos tubos sem abrir janela, captura uma vista e mostra 3
imagens: RGB, profundidade (em metros) e máscara de segmentação.
A figura também é salva em data/camera_vista.png.
"""
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pybullet as p

from percepcao.simulacao import camera, cenas

RAIZ = Path(__file__).resolve().parent.parent


if __name__ == "__main__":
    p.connect(p.DIRECT)          # sem janela: só precisamos das imagens
    gt = cenas.montar_tubos()
    foto = camera.capturar(camera.OLHO, camera.ALVO)

    prof, masc = foto["profundidade"], foto["mascara"]
    print(f"Intrínsecos: {foto['intrinsecos']}")

    nomes = {pr["id_pybullet"]: pr["nome"] for pr in gt["primitivas"]}
    nomes[-1] = "fundo (céu)"
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
    saida.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(saida, dpi=120)
    print(f"\nFigura salva em {saida}")
    plt.show()
