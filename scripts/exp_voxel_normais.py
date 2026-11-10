"""
Experimentos do voxel e das normais, sempre contra o ground truth.

Rodar (com o venv ativo):
    python scripts/gerar_teste.py         (uma vez, se data/teste/ não existir)
    python scripts/exp_voxel_normais.py

Três experimentos:
  A. Voxel: para vários tamanhos de cubinho, quantos pontos sobram de cada
     objeto, quanto tempo leva e quanto o voxel "achata" os tubos.
  B. Normais: para vários raios de vizinhança, o erro angular contra as
     normais verdadeiras e quanto tempo leva.
  C. Voxel x raio no tubo mais fino: a combinação dos dois.

Salva em data/resultados/: tabelas .csv e gráficos .png.
"""
import csv
import time
from pathlib import Path

import matplotlib
matplotlib.use("Agg")            # só salva os gráficos, não abre janela
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np               # noqa: E402
import open3d as o3d             # noqa: E402

from percepcao.avaliacao import resumo_normais         # noqa: E402
from percepcao.etapas.filtros import reduzir_voxel     # noqa: E402
from percepcao.etapas.normais import estimar_normais   # noqa: E402
from percepcao.nuvem import Nuvem, carregar_gt         # noqa: E402

RAIZ = Path(__file__).resolve().parent.parent

TESTE = RAIZ / "data" / "teste"
SAIDA = RAIZ / "data" / "resultados"

VOXELS_CM = [0.5, 1, 2, 3, 5, 8]          # tamanhos de cubinho testados
RAIOS_CM = [1, 2, 3, 5, 8]                # raios de vizinhança das normais
REPETICOES = 5                            # o plano pede média de várias medidas

# Cores fixas por objeto (paleta validada para daltonismo)
CORES = {"piso": "#2a78d6", "pilar": "#eb6834",
         "tubo_piso": "#1baf7a", "tubo_elevado": "#eda100", "galho": "#eb6834"}
MARCAS = {"piso": "o", "pilar": "s", "tubo_piso": "^", "tubo_elevado": "D",
          "galho": "s"}


# Roda a função várias vezes e devolve o tempo médio em milissegundos
def cronometrar(funcao, repeticoes=REPETICOES):
    tempos = []
    for _ in range(repeticoes):
        t0 = time.perf_counter()
        resultado = funcao()
        tempos.append(time.perf_counter() - t0)
    return resultado, 1000 * float(np.mean(tempos))


# Quanto o voxel achata cada cilindro: a mediana de (distância ao eixo - raio)
# dos pontos da lateral. Negativo = os pontos foram puxados para dentro.
# (O voxel troca vários pontos de um arco pela média deles, e a média de
# pontos num arco fica um pouco para dentro da curva.)
def desvio_raio_mm(nuvem, pr):
    sel = nuvem.pontos[nuvem.rotulos == pr["id_pybullet"]]
    if len(sel) == 0:
        return float("nan")
    eixo, centro = np.array(pr["eixo"]), np.array(pr["centro"])
    rel = sel - centro
    t = rel @ eixo
    lateral = np.abs(t) < pr["altura"] / 2 - 0.02
    dist = np.linalg.norm(rel - np.outer(t, eixo), axis=1)[lateral]
    return float(np.median(dist - pr["raio"]) * 1000)


def salvar_csv(linhas, nome):
    with open(SAIDA / nome, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(linhas[0].keys()))
        w.writeheader()
        w.writerows(linhas)


def estilo(ax, titulo, xlabel, ylabel):
    ax.set_title(titulo, loc="left", fontsize=11)
    ax.set_xlabel(xlabel)
    ax.set_ylabel(ylabel)
    ax.grid(True, color="#e6e6e3", linewidth=0.8)
    ax.set_axisbelow(True)
    for lado in ["top", "right"]:
        ax.spines[lado].set_visible(False)


# ---------------------------------------------------------------------------
# A. Voxel
# ---------------------------------------------------------------------------
def experimento_voxel(nuvem, gt):
    cilindros = [pr for pr in gt["primitivas"] if pr["tipo"] == "cilindro"]
    nomes = {pr["id_pybullet"]: pr["nome"] for pr in gt["primitivas"]}
    linhas = []

    entrada = nuvem.contagem()
    linhas.append({"voxel_cm": 0, "tempo_ms": 0.0, "total": len(nuvem),
                   **{nomes[i]: entrada.get(i, 0) for i in nomes},
                   **{f"desvio_{c['nome']}_mm": desvio_raio_mm(nuvem, c)
                      for c in cilindros}})

    for v in VOXELS_CM:
        # O tempo medido é só o do Open3D (o que rodaria no robô); a conta
        # dos rótulos é da avaliação, não do filtro
        pcd = nuvem.para_o3d()
        _, ms = cronometrar(lambda: pcd.voxel_down_sample(v / 100))
        red = reduzir_voxel(nuvem, tamanho=v / 100)
        cont = red.contagem()
        linhas.append({"voxel_cm": v, "tempo_ms": ms, "total": len(red),
                       **{nomes[i]: cont.get(i, 0) for i in nomes},
                       **{f"desvio_{c['nome']}_mm": desvio_raio_mm(red, c)
                          for c in cilindros}})
    return linhas


# Quantos vizinhos um ponto típico tem dentro do raio (numa amostra de
# 1 a cada 500 pontos, para ser rápido)
def vizinhos_medianos(nuvem, raio):
    pcd = nuvem.para_o3d()
    arvore = o3d.geometry.KDTreeFlann(pcd)
    k = [arvore.search_radius_vector_3d(pcd.points[i], raio)[0]
         for i in range(0, len(nuvem), 500)]
    return int(np.median(k))


# ---------------------------------------------------------------------------
# B. Normais (sobre a nuvem já reduzida com voxel de 1 cm)
# ---------------------------------------------------------------------------
def experimento_normais(nuvem, gt, rotulo_cena):
    linhas = []
    for r in RAIOS_CM:
        com_normais, ms = cronometrar(
            lambda: estimar_normais(nuvem, raio=r / 100), repeticoes=3)
        viz = vizinhos_medianos(nuvem, r / 100)
        for nome, res in resumo_normais(com_normais, gt).items():
            linhas.append({"cena": rotulo_cena, "raio_cm": r, "objeto": nome,
                           "vizinhos": viz, "tempo_ms": ms, **res})
    return linhas


# ---------------------------------------------------------------------------
# C. Voxel x raio, no tubo mais fino (r = 5 cm)
# ---------------------------------------------------------------------------
def experimento_grade(nuvem, gt, objeto="tubo_elevado"):
    tabela = {}
    for v in [0.5, 1, 2, 3]:
        red = reduzir_voxel(nuvem, tamanho=v / 100)
        for r in [2, 3, 5, 8]:
            res = resumo_normais(estimar_normais(red, raio=r / 100), gt)
            tabela[(v, r)] = res.get(objeto, {}).get("erro_mediano", np.nan)
    return tabela


if __name__ == "__main__":
    SAIDA.mkdir(parents=True, exist_ok=True)
    gt_tubos = carregar_gt(TESTE / "tubos_gt.json")
    gt_galho = carregar_gt(TESTE / "galho_gt.json")
    tubos = Nuvem.carregar(TESTE / "tubos_3vistas.ply")
    galho = Nuvem.carregar(TESTE / "galho_alt15.ply")

    # --- A ------------------------------------------------------------------
    print("A. VOXEL (cena dos tubos, 3 vistas)\n")
    A = experimento_voxel(tubos, gt_tubos)
    salvar_csv(A, "voxel.csv")
    print(f"{'voxel':>7s} {'tempo':>8s} {'total':>8s} {'piso':>8s} {'pilar':>7s} "
          f"{'tubo_piso':>9s} {'tubo_elev':>9s} | achatamento (mm): "
          f"{'pilar':>6s} {'t_piso':>6s} {'t_elev':>6s}")
    for l in A:
        print(f"{l['voxel_cm']:5.1f}cm {l['tempo_ms']:6.0f}ms {l['total']:8d} "
              f"{l['piso']:8d} {l['pilar']:7d} {l['tubo_piso']:9d} "
              f"{l['tubo_elevado']:9d} |                   "
              f"{l['desvio_pilar_mm']:6.2f} {l['desvio_tubo_piso_mm']:6.2f} "
              f"{l['desvio_tubo_elevado_mm']:6.2f}")

    # --- B ------------------------------------------------------------------
    print("\nB. NORMAIS (depois do voxel de 1 cm)\n")
    B = (experimento_normais(reduzir_voxel(tubos, 0.01), gt_tubos, "tubos")
         + experimento_normais(reduzir_voxel(galho, 0.01), gt_galho, "galho_alt15"))
    salvar_csv(B, "normais.csv")
    print(f"{'cena':12s} {'raio':>5s} {'vizinhos':>8s} {'objeto':13s} "
          f"{'erro mediano':>13s} {'erro p90':>9s} {'sentido ok':>11s} {'tempo':>8s}")
    for l in B:
        print(f"{l['cena']:12s} {l['raio_cm']:3d}cm {l['vizinhos']:8d} {l['objeto']:13s} "
              f"{l['erro_mediano']:11.2f}° {l['erro_p90']:8.2f}° "
              f"{l['sentido_certo_%']:10.1f}% {l['tempo_ms']:6.0f}ms")

    # --- C ------------------------------------------------------------------
    print("\nC. ERRO MEDIANO DAS NORMAIS NO TUBO MAIS FINO (r = 5 cm), em graus\n")
    C = experimento_grade(tubos, gt_tubos)
    raios = sorted({r for _, r in C})
    print("voxel \\ raio " + "".join(f"{r:>7d}cm" for r in raios))
    for v in sorted({v for v, _ in C}):
        print(f"{v:9.1f} cm  " + "".join(f"{C[(v, r)]:8.2f}°" for r in raios))

    # --- Gráficos -----------------------------------------------------------
    fig, ax = plt.subplots(1, 2, figsize=(12, 4.5))

    objs = ["piso", "pilar", "tubo_piso", "tubo_elevado"]
    vx = [l["voxel_cm"] for l in A[1:]]
    for o in objs:
        y = [l[o] for l in A[1:]]
        ax[0].plot(vx, y, color=CORES[o], marker=MARCAS[o], markersize=8,
                   linewidth=2, label=o)
        ax[0].annotate(o, (vx[-1], y[-1]), xytext=(6, 0),
                       textcoords="offset points", va="center", fontsize=9,
                       color="#3d3d3a")
    ax[0].set_yscale("log")
    ax[0].set_xlim(0, max(vx) * 1.35)
    estilo(ax[0], "Pontos que sobram de cada objeto", "tamanho do voxel (cm)",
           "pontos (escala log)")
    ax[0].legend(frameon=False, fontsize=9)

    for o in objs:
        sel = [l for l in B if l["cena"] == "tubos" and l["objeto"] == o]
        x = [l["raio_cm"] for l in sel]
        y = [l["erro_mediano"] for l in sel]
        ax[1].plot(x, y, color=CORES[o], marker=MARCAS[o], markersize=8,
                   linewidth=2, label=o)
        ax[1].annotate(o, (x[-1], y[-1]), xytext=(6, 0),
                       textcoords="offset points", va="center", fontsize=9,
                       color="#3d3d3a")
    ax[1].set_xlim(0, max(RAIOS_CM) * 1.35)
    estilo(ax[1], "Erro das normais contra o GT (voxel 1 cm)",
           "raio de vizinhança (cm)", "erro angular mediano (graus)")
    ax[1].legend(frameon=False, fontsize=9)

    fig.tight_layout()
    fig.savefig(SAIDA / "voxel_normais.png", dpi=130)
    print(f"\nResultados em {SAIDA}")
