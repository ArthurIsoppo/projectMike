"""
Gera o CONJUNTO DE TESTE FIXO: as nuvens sobre as quais todos os
experimentos rodam. O plano pede que todo experimento use os mesmos
arquivos, então este script roda uma vez e os arquivos ficam congelados em
data/teste/ (o gerar_nuvem.py e o aranha.py continuam livres para brincar).

Rodar (com o venv ativo):
    python scripts/gerar_teste.py

Gera em data/teste/:
    tubos_1vista.ply, tubos_3vistas.ply     cena dos 3 tubos (cenas.montar_tubos)
    galho_alt05.ply, _alt15, _alt30         a aranha a 5, 15 e 30 cm do galho
    (cada .ply com o seu _rotulos.npy e _origens.npy)
    tubos_gt.json, galho_gt.json            ground truth, com TODAS as
                                            primitivas: piso (plano) + cilindros
"""
from pathlib import Path

import pybullet as p

from percepcao.simulacao import cenas, trajetoria
from percepcao.simulacao.projecao import fotografar

PASTA = Path(__file__).resolve().parent.parent / "data" / "teste"


if __name__ == "__main__":
    PASTA.mkdir(parents=True, exist_ok=True)
    p.connect(p.DIRECT)

    # --- Cena dos 3 tubos -----------------------------------------------
    gt = cenas.montar_tubos()
    cenas.salvar_gt(gt, PASTA / "tubos_gt.json")
    for n, nome in [(1, "tubos_1vista"), (3, "tubos_3vistas")]:
        nv = fotografar(cenas.VISTAS_TUBOS[:n])
        nv.salvar(PASTA / f"{nome}.ply")
        print(f"{nome:15s} {len(nv):7d} pontos  {nv.contagem()}")

    # --- Galho: a aranha parada a 2 m da base, em 3 alturas --------------
    # (a câmera sempre a 45° para baixo: frente = altura)
    p.resetSimulation()
    gt = cenas.montar_galho()
    cenas.salvar_gt(gt, PASTA / "galho_gt.json")
    cil = [pr for pr in gt["primitivas"] if pr["tipo"] == "cilindro"][0]
    for alt in [0.05, 0.15, 0.30]:
        olho, alvo, cima = trajetoria.pose_aranha(cil, 2.0, altura=alt, frente=alt)
        nv = fotografar([(olho, alvo, cima)], near=trajetoria.NEAR,
                        prof_max=trajetoria.PROF_MAX)
        nome = f"galho_alt{round(alt * 100):02d}"
        nv.salvar(PASTA / f"{nome}.ply")
        print(f"{nome:15s} {len(nv):7d} pontos  {nv.contagem()}")

    print(f"\nConjunto de teste salvo em {PASTA}")
