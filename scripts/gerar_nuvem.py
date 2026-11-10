"""
Nuvem de pontos da cena dos tubos: fotografa, confere contra o ground truth,
salva em .ply e abre o visualizador do Open3D.

Rodar (com o venv ativo):
    python scripts/gerar_nuvem.py       usa todas as vistas de cenas.VISTAS_TUBOS
    python scripts/gerar_nuvem.py 1     usa só a primeira vista (bom para comparar)

Gera em data/:
    nuvem.ply    pontos 3D (no referencial do MUNDO) com cor
    rotulos.npy  para cada ponto, o id_pybullet do objeto (0 = piso, 1-3 = cilindros)
    origens.npy  para cada ponto, a posição da câmera que o viu
    gt.json      ground truth da cena (piso + cilindros)
"""
import sys
from pathlib import Path

import open3d as o3d
import pybullet as p

from percepcao.avaliacao import conferir
from percepcao.simulacao import cenas
from percepcao.simulacao.projecao import fotografar

PASTA = Path(__file__).resolve().parent.parent / "data"


if __name__ == "__main__":
    p.connect(p.DIRECT)
    gt = cenas.montar_tubos()
    cenas.salvar_gt(gt, PASTA / "gt.json")

    # Quantas vistas usar: o número passado no terminal, ou todas.
    # sys.argv é a lista do que foi digitado: ["scripts/gerar_nuvem.py", "1"]
    n_vistas = int(sys.argv[1]) if len(sys.argv) > 1 else len(cenas.VISTAS_TUBOS)
    nuvem = fotografar(cenas.VISTAS_TUBOS[:n_vistas])

    print("\nConferência contra o ground truth:")
    conferir(nuvem.pontos, nuvem.rotulos, gt)

    nuvem.salvar(PASTA / "nuvem.ply")
    print(f"\nSalvo: {PASTA / 'nuvem.ply'} ({len(nuvem)} pontos), "
          f"rotulos.npy e origens.npy")

    # Abre o visualizador do Open3D. Mouse: botão esquerdo gira, roda dá zoom,
    # Ctrl + esquerdo move. Os eixos coloridos têm 0,5 m (X vermelho,
    # Y verde, Z azul) e ficam na origem do mundo.
    eixos = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.5)
    o3d.visualization.draw_geometries([nuvem.para_o3d(), eixos],
                                      window_name="Nuvem de pontos")
