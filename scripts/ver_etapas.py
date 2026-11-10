"""
Teste rápido das etapas: passa a nuvem pelas 3 etapas e mostra quantos
pontos de cada objeto sobram depois de cada uma.

Rodar (com o venv ativo):
    python scripts/gerar_teste.py      (uma vez, se data/teste/ não existir)
    python scripts/ver_etapas.py

Usa data/teste/tubos_3vistas.ply.
"""
from pathlib import Path

from percepcao.etapas.filtros import reduzir_voxel, remover_outliers
from percepcao.etapas.normais import estimar_normais
from percepcao.nuvem import Nuvem, carregar_gt

PASTA = Path(__file__).resolve().parent.parent / "data" / "teste"


if __name__ == "__main__":
    gt = carregar_gt(PASTA / "tubos_gt.json")
    nomes = {pr["id_pybullet"]: pr["nome"] for pr in gt["primitivas"]}

    nuvem = Nuvem.carregar(PASTA / "tubos_3vistas.ply")
    etapas = [
        ("entrada", None),
        ("outliers", lambda n: remover_outliers(n)),
        ("voxel 1 cm", lambda n: reduzir_voxel(n, tamanho=0.01)),
        ("normais", lambda n: estimar_normais(n)),
    ]

    print(f"{'etapa':12s}" + "".join(f"{nomes[i]:>14s}" for i in nomes)
          + f"{'total':>10s}")
    for nome_etapa, etapa in etapas:
        if etapa is not None:
            nuvem = etapa(nuvem)
        cont = nuvem.contagem()
        print(f"{nome_etapa:12s}"
              + "".join(f"{cont.get(i, 0):14d}" for i in nomes)
              + f"{len(nuvem):10d}")
