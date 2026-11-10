"""
Roda o pipeline inteiro numa nuvem do conjunto de teste e mede cada etapa:
tempo médio (com desvio-padrão) e quantos pontos de cada objeto sobram.

Rodar (com o venv ativo):
    python scripts/gerar_teste.py          (uma vez, se data/teste/ não existir)
    python scripts/rodar_pipeline.py                         # tubos_3vistas
    python scripts/rodar_pipeline.py galho_alt15
    python scripts/rodar_pipeline.py --sem outliers          # desliga etapas
    python scripts/rodar_pipeline.py --sem outliers,normais galho_alt15
    python scripts/rodar_pipeline.py --repeticoes 10
    python scripts/rodar_pipeline.py --ver                   # abre o Open3D no fim

A nuvem vem de data/teste/<nome>.ply, e o GT é deduzido pelo começo do nome
(tubos_* -> tubos_gt.json, galho_* -> galho_gt.json).

Imprime a tabela do relatório e, fora do tempo medido, o erro das normais
contra o GT. Salva o relatório em data/resultados/pipeline_<nome>.csv, com
as etapas do --sem no nome (pipeline_tubos_3vistas_sem-outliers.csv), para
uma rodada com etapas desligadas não apagar o relatório do pipeline completo.

--sem recebe as etapas separadas por vírgula, sem espaço (--sem outliers,voxel),
então a ordem dos argumentos não importa.

Tempos marcados com * vêm da funcao_tempo da etapa: só o trabalho real, sem a
contabilidade dos rótulos do GT (ver a seção 2 de percepcao/pipeline.py).

Com --ver, DEPOIS de medir tudo, abre o Open3D com a nuvem bruta (em cinza,
deslocada em X) ao lado da filtrada (colorida por objeto).
"""
import argparse
import csv
import sys
from pathlib import Path

import numpy as np
import open3d as o3d

from percepcao.avaliacao import resumo_normais
from percepcao.nuvem import Nuvem, carregar_gt
from percepcao.pipeline import PIPELINE_PADRAO, rodar, sem

RAIZ = Path(__file__).resolve().parent.parent
TESTE = RAIZ / "data" / "teste"
SAIDA = RAIZ / "data" / "resultados"

# Cores fixas por objeto, as mesmas dos gráficos do exp_voxel_normais.py
# (paleta validada para daltonismo). Objeto fora da lista: cinza escuro.
CORES = {"piso": "#2a78d6", "pilar": "#eb6834", "tubo_piso": "#1baf7a",
         "tubo_elevado": "#eda100", "galho": "#eb6834"}
COR_OUTRO = "#555555"
COR_BRUTA = "#9a9a9a"


def ler_argumentos():
    nomes_etapas = [e.nome for e in PIPELINE_PADRAO]
    ap = argparse.ArgumentParser(
        description="Roda o pipeline numa nuvem de data/teste/ e mede cada etapa.")
    ap.add_argument("nuvem", nargs="?", default="tubos_3vistas",
                    help="nome do .ply em data/teste/, sem a extensão "
                         "(padrão: tubos_3vistas)")
    # --sem é UM argumento só ("outliers,voxel"), que o type= quebra na
    # vírgula. Com nargs="+" (várias palavras), o argparse engoliria o nome
    # da nuvem que viesse depois como se fosse mais uma etapa.
    def lista_etapas(texto):
        etapas = [t.strip() for t in texto.split(",") if t.strip()]
        erradas = [e for e in etapas if e not in nomes_etapas]
        if erradas:
            raise argparse.ArgumentTypeError(
                f"etapa(s) desconhecida(s): {', '.join(erradas)} "
                f"(as etapas são: {', '.join(nomes_etapas)})")
        return etapas

    ap.add_argument("--sem", type=lista_etapas, default=[],
                    metavar="ETAPA[,ETAPA...]",
                    help=f"etapas a desligar, separadas por vírgula "
                         f"({', '.join(nomes_etapas)})")
    ap.add_argument("--repeticoes", type=int, default=5,
                    help="execuções medidas por etapa (padrão: 5)")
    ap.add_argument("--ver", action="store_true",
                    help="no fim, abre o Open3D com a nuvem bruta e a filtrada")
    args = ap.parse_args()
    if args.repeticoes < 1:
        ap.error("--repeticoes tem que ser pelo menos 1")
    return args


# tubos_3vistas + --sem voxel,outliers -> pipeline_tubos_3vistas_sem-outliers-voxel.csv
# (as etapas na ordem do pipeline, para o nome não depender da ordem digitada)
def caminho_csv(nome_nuvem, desligadas):
    ordem = [e.nome for e in PIPELINE_PADRAO if e.nome in desligadas]
    sufixo = f"_sem-{'-'.join(ordem)}" if ordem else ""
    return SAIDA / f"pipeline_{nome_nuvem}{sufixo}.csv"


# tubos_3vistas -> tubos_gt.json; galho_alt15 -> galho_gt.json
def caminho_gt(nome_nuvem):
    return TESTE / f"{nome_nuvem.split('_')[0]}_gt.json"


# A tabela tem uma linha "entrada" (a nuvem bruta, sem tempo) e uma por
# etapa. Os objetos aparecem pelo nome do GT, não pelo id do PyBullet.
# Um * no tempo = medido com a funcao_tempo (só o trabalho real).
def imprimir_tabela(nuvem, relatorio, nomes):
    print(f"{'etapa':12s}{'tempo (ms)':>18s}{'total':>9s}"
          + "".join(f"{nomes[i]:>14s}" for i in nomes))

    cont = nuvem.contagem()
    print(f"{'entrada':12s}{'':>18s}{len(nuvem):9d}"
          + "".join(f"{cont.get(i, 0):14d}" for i in nomes))

    for r in relatorio:
        if not r["ligada"]:
            print(f"{r['etapa']:12s}{'(desligada)':>18s}")
            continue
        marca = "*" if r["tempo_de"] == "funcao_tempo" else " "
        tempo = f"{r['tempo_ms']:.1f} ± {r['desvio_ms']:.1f}{marca}"
        print(f"{r['etapa']:12s}{tempo:>18s}{r['pontos_saida']:9d}"
              + "".join(f"{r['por_objeto'].get(i, 0):14d}" for i in nomes))

    if any(r["tempo_de"] == "funcao_tempo" for r in relatorio):
        print("\n* tempo só do trabalho real (funcao_tempo), sem a "
              "contabilidade dos rótulos do GT")


# Uma linha do CSV por etapa; os pontos por objeto viram uma coluna cada.
# Etapa desligada fica com as células vazias.
def salvar_csv(relatorio, nomes, caminho):
    caminho.parent.mkdir(parents=True, exist_ok=True)
    colunas = ["etapa", "ligada", "tempo_ms", "desvio_ms", "tempo_de",
               "pontos_entrada", "pontos_saida"] + list(nomes.values())
    with open(caminho, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=colunas)
        w.writeheader()
        for r in relatorio:
            linha = {c: r[c] for c in colunas[:7]}
            por_objeto = r["por_objeto"] or {}
            for i, nome in nomes.items():
                linha[nome] = por_objeto.get(i) if r["ligada"] else None
            w.writerow(linha)


# "#2a78d6" -> [0.16, 0.47, 0.84]: o Open3D quer as cores de 0 a 1
def rgb(hexa):
    return [int(hexa[i:i + 2], 16) / 255 for i in (1, 3, 5)]


# Monta as duas nuvens para o Open3D, lado a lado:
#   - a filtrada no lugar, com a cor de cada objeto (pelo rótulo do GT);
#   - a bruta em cinza, deslocada em X pela largura da cena mais uma folga
#     de 20%, para não ficar uma em cima da outra. Como é uma cópia
#     deslocada, as duas aparecem com o mesmo zoom e ângulo.
# Separado do main para dar para conferir sem abrir a janela.
def montar_vista(bruta, final, nomes):
    filtrada = final.para_o3d()
    cores = np.array([rgb(COR_OUTRO)] * len(final))
    for i, nome in nomes.items():
        cores[final.rotulos == i] = rgb(CORES.get(nome, COR_OUTRO))
    filtrada.colors = o3d.utility.Vector3dVector(cores)

    largura = bruta.pontos[:, 0].max() - bruta.pontos[:, 0].min()
    deslocada = bruta.pontos + [1.2 * largura, 0.0, 0.0]
    vista_bruta = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(deslocada))
    vista_bruta.paint_uniform_color(rgb(COR_BRUTA))
    return [filtrada, vista_bruta]


if __name__ == "__main__":
    args = ler_argumentos()

    arq_nuvem = TESTE / f"{args.nuvem}.ply"
    arq_gt = caminho_gt(args.nuvem)
    for arq in (arq_nuvem, arq_gt):
        if not arq.exists():
            sys.exit(f"Não achei {arq.relative_to(RAIZ)}. "
                     f"Rode antes: python scripts/gerar_teste.py")

    gt = carregar_gt(arq_gt)
    nomes = {pr["id_pybullet"]: pr["nome"] for pr in gt["primitivas"]}
    bruta = Nuvem.carregar(arq_nuvem)
    etapas = sem(PIPELINE_PADRAO, *args.sem)

    print(f"Nuvem: {arq_nuvem.relative_to(RAIZ)}  |  {args.repeticoes} "
          f"repetições por etapa (+1 de aquecimento)\n")

    # 1. Medir (só isto conta tempo)
    final, primitivas, relatorio = rodar(bruta, etapas,
                                         repeticoes=args.repeticoes)

    # 2. Relatório
    imprimir_tabela(bruta, relatorio, nomes)
    arq_csv = caminho_csv(args.nuvem, args.sem)
    salvar_csv(relatorio, nomes, arq_csv)
    print(f"\nRelatório salvo em {arq_csv.relative_to(RAIZ)}")

    # 3. Erro das normais contra o GT, fora do tempo medido
    if final.normais is None:
        print("\n(sem normais na nuvem final: a etapa normais está desligada)")
    else:
        print("\nErro das normais (direção, em graus):\n")
        print(f"{'objeto':14s}{'pontos':>9s}{'mediana':>9s}{'p90':>8s}"
              f"{'sentido certo':>15s}")
        for nome, res in resumo_normais(final, gt).items():
            print(f"{nome:14s}{res['pontos']:9d}{res['erro_mediano']:8.2f}°"
                  f"{res['erro_p90']:7.2f}°{res['sentido_certo_%']:14.1f}%")

    if primitivas is not None:
        print(f"\n{len(primitivas)} primitivas ajustadas")

    # 4. Visualização, só agora: a medição já terminou e não é afetada
    if args.ver:
        print("\nAbrindo o Open3D: filtrada (colorida) à esquerda, bruta "
              "(cinza) à direita. Feche a janela para terminar.")
        o3d.visualization.draw_geometries(
            montar_vista(bruta, final, nomes),
            window_name=f"pipeline: {args.nuvem} (filtrada | bruta)")
