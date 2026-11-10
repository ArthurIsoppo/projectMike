"""
A aranha: uma câmera que escala o galho, andando ao longo do eixo do tubo,
um pouco acima da superfície, e tirando uma foto a cada passo.

Rodar (com o venv ativo):
    python scripts/aranha.py              sem janela, só gera os quadros
    python scripts/aranha.py gui          abre a janela e mostra a aranha subindo

Os parâmetros (altura, trecho, número de quadros...) ficam em
percepcao/simulacao/trajetoria.py.

Gera em data/galho/:
    quadro_000.ply, quadro_001.ply, ...   nuvem de cada passo (referencial do MUNDO)
    quadro_000_rotulos.npy, ...           objeto de cada ponto (0 = piso, 1 = galho)
    quadro_000_origens.npy, ...           posição da câmera que viu cada ponto
    gt.json                               ground truth (piso + galho)
    trajetoria.json                       pose da câmera em cada quadro
"""
import json
import math
import sys
import time
from pathlib import Path

import numpy as np
import pybullet as p

from percepcao.avaliacao import medir_quadro
from percepcao.simulacao import cenas
from percepcao.simulacao.projecao import fotografar
from percepcao.simulacao.trajetoria import (ALTURA, FIM, INICIO, N_QUADROS,
                                            NEAR, PROF_MAX, pose_aranha)

PASTA = Path(__file__).resolve().parent.parent / "data" / "galho"


# ---------------------------------------------------------------------------
# Desenho da aranha na janela (só no modo gui)
#
# Usamos linhas de debug, e não um objeto de verdade, porque linhas de debug
# não aparecem na foto da câmera. Um objeto no lugar da câmera taparia a
# visão dela.
# ---------------------------------------------------------------------------
def desenhar_aranha(olho, alvo, cima, ids):
    a = 0.08                                    # tamanho das "patas"
    linhas = [
        (olho, alvo, [1, 0.8, 0]),               # amarelo: para onde olha
        (olho, olho - cima * (ALTURA * 0.9), [0, 0, 0]),   # preto: até o tubo
        (olho - [a, 0, 0], olho + [a, 0, 0], [0, 0, 0]),
        (olho - [0, a, 0], olho + [0, a, 0], [0, 0, 0]),
    ]
    novos = []
    for k, (ini, fim, cor) in enumerate(linhas):
        kwargs = {"replaceItemUniqueId": ids[k]} if ids else {}
        novos.append(p.addUserDebugLine(list(ini), list(fim), cor,
                                        lineWidth=3, **kwargs))
    return novos


# ---------------------------------------------------------------------------
# Junta tudo: a aranha sobe o galho, quadro a quadro
# ---------------------------------------------------------------------------
if __name__ == "__main__":
    com_janela = "gui" in sys.argv

    p.connect(p.GUI if com_janela else p.DIRECT)
    gt = cenas.montar_galho()
    cil = [pr for pr in gt["primitivas"] if pr["tipo"] == "cilindro"][0]

    PASTA.mkdir(parents=True, exist_ok=True)
    cenas.salvar_gt(gt, PASTA / "gt.json")

    D = cil["raio"] + ALTURA
    arco_teorico = 2 * math.degrees(math.acos(cil["raio"] / D))
    print(f"Altura da câmera: {ALTURA * 100:.0f} cm acima do tubo; "
          f"arco máximo visível (teórico): {arco_teorico:.0f}°\n")

    if com_janela:
        # Câmera da janela PARADA, vendo o galho inteiro de lado, para dar
        # para ver a aranha subindo (se ela seguisse a aranha, a aranha
        # ficaria sempre no meio da tela e pareceria parada)
        p.resetDebugVisualizerCamera(cameraDistance=5.5, cameraYaw=0,
                                     cameraPitch=-10,
                                     cameraTargetPosition=cil["centro"])

    posicoes = np.linspace(INICIO, FIM, N_QUADROS)
    trajetoria, ids_desenho = [], None
    for i, s in enumerate(posicoes):
        olho, alvo, cima = pose_aranha(cil, s)

        if com_janela:
            # Anda suave do quadro anterior até este, em 10 passinhos.
            # Só desenha: a foto é tirada uma vez, no fim do caminho.
            s_antes = posicoes[i - 1] if i > 0 else s
            for s_meio in np.linspace(s_antes, s, 10):
                o, a, c = pose_aranha(cil, s_meio)
                ids_desenho = desenhar_aranha(o, a, c, ids_desenho)
                time.sleep(0.03)
            # Rastro: um pontinho vermelho onde cada foto foi tirada
            p.addUserDebugPoints([olho.tolist()], [[1, 0, 0]], pointSize=6)

        nuvem = fotografar([(olho, alvo, cima)], near=NEAR, prof_max=PROF_MAX)
        nuvem.salvar(PASTA / f"quadro_{i:03d}.ply")

        m = medir_quadro(nuvem.pontos, nuvem.rotulos, cil, cima)
        trajetoria.append({"quadro": i, "s": float(s), "olho": olho.tolist(),
                           "alvo": alvo.tolist(), "cima": cima.tolist(),
                           "medidas": m})
        if m:
            print(f"quadro {i:2d}  s = {s:4.2f} m | {len(nuvem):6d} pts "
                  f"(galho {m['pontos_galho']:6d}) | arco {m['arco_graus']:3.0f}° | "
                  f"trecho {m['trecho_m'][0]:4.2f}-{m['trecho_m'][1]:4.2f} m | "
                  f"raio {m['raio_mediano_mm']:5.1f} mm")
        else:
            print(f"quadro {i:2d}  s = {s:4.2f} m | o galho não aparece!")

    with open(PASTA / "trajetoria.json", "w") as f:
        json.dump(trajetoria, f, indent=2)
    print(f"\nSalvo em {PASTA}: {N_QUADROS} quadros, gt.json e trajetoria.json")

    if com_janela:
        print("Feche a janela para terminar.")
        while p.isConnected():
            time.sleep(0.1)
