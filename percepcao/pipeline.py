"""
Pipeline integrado: encadeia as etapas, sempre de Nuvem em Nuvem (ver o
combinado em percepcao/nuvem.py), e mede cada uma.

    outliers -> voxel -> normais -> primitivas
    (filtros)  (filtros) (normais)  (primitivas, desligada até existir)

Cada etapa é um objeto Etapa (nome, função, parâmetros, ligada?), então
trocar ou desligar uma etapa é só mexer na lista, sem reescrever o código.

Uso:
    from percepcao.pipeline import PIPELINE_PADRAO, rodar, sem
    final, primitivas, relatorio = rodar(nuvem, PIPELINE_PADRAO)
    final, _, relatorio = rodar(nuvem, sem(PIPELINE_PADRAO, "outliers"))

Para rodar pela linha de comando, ver scripts/rodar_pipeline.py.
"""
import time
from dataclasses import dataclass, field, replace
from typing import Callable

import numpy as np

from percepcao.etapas.filtros import reduzir_voxel, remover_outliers
from percepcao.etapas.normais import estimar_normais
from percepcao.etapas.primitivas import ajustar
from percepcao.nuvem import Nuvem


# ---------------------------------------------------------------------------
# 1. A Etapa
#
#   nome:       como ela aparece no relatório e no --sem do script
#   funcao:     a função da etapa, chamada como funcao(nuvem, **parametros)
#   parametros: os valores escolhidos (por exemplo {"tamanho": 0.01})
#   ligada:     False pula a etapa (ela aparece no relatório como desligada)
#   devolve_primitivas: True só na etapa final, que devolve uma LISTA de
#               primitivas (regra 3 do combinado) em vez de uma Nuvem.
#               É um campo explícito para não ter que adivinhar pelo que a
#               função devolveu: se um filtro com bug devolver uma lista,
#               queremos um erro, não "ah, deve ser o ajuste".
#   funcao_tempo: opcional. Uma versão da etapa que faz SÓ o trabalho real,
#               sem rótulos e sem origens, chamada do mesmo jeito
#               (funcao_tempo(nuvem, **parametros)). Ver a seção 2.
#
# field(default_factory=dict): cada Etapa ganha o SEU dicionário vazio.
# Com "parametros: dict = {}", todas dividiriam o mesmo (uma armadilha
# clássica do Python).
# ---------------------------------------------------------------------------
@dataclass
class Etapa:
    nome: str
    funcao: Callable
    parametros: dict = field(default_factory=dict)
    ligada: bool = True
    devolve_primitivas: bool = False
    funcao_tempo: Callable | None = None


# ---------------------------------------------------------------------------
# 2. Custo real x custo de avaliação
#
# Os rótulos (o GT por ponto) e as origens só existem porque a cena é
# simulada: o robô de verdade recebe só pontos. Mas as nossas etapas carregam
# os rótulos junto, para dar para medir o que cada uma fez com cada objeto.
#
# Na maioria das etapas isso não custa quase nada (selecionar() só indexa
# mais um vetor). No voxel custa MUITO: para dar um rótulo a cada cubinho, o
# reduzir_voxel precisa saber quais pontos caíram em cada um (a versão
# "_and_trace") e contar os rótulos, e isso leva a maior parte do tempo da
# etapa. Esse é um custo de AVALIAÇÃO, não do filtro.
#
# Por isso a etapa pode ter uma funcao_tempo: o rodar() cronometra ela, e a
# saída que segue pelo pipeline continua vindo da funcao (com rótulos). O
# relatório marca de onde veio cada tempo, para a tabela não enganar.
#
# Atenção: a funcao_tempo NÃO é idêntica à funcao, só equivalente em custo.
# A conversão para o Open3D (para_o3d) fica dentro do tempo, porque o robô
# também teria que montar a nuvem do Open3D a partir dos pontos do sensor.
# ---------------------------------------------------------------------------
def voxel_sem_rotulos(nuvem, tamanho=0.01):
    return nuvem.para_o3d().voxel_down_sample(tamanho)


# Os valores escolhidos nos experimentos (scripts/exp_voxel_normais.py):
# voxel de 1 cm e raio de 3 cm para as normais.
PIPELINE_PADRAO = [
    Etapa("outliers", remover_outliers, {"vizinhos": 20, "desvios": 2.0}),
    Etapa("voxel", reduzir_voxel, {"tamanho": 0.01},
          funcao_tempo=voxel_sem_rotulos),
    Etapa("normais", estimar_normais, {"raio": 0.03}),
    Etapa("primitivas", ajustar, ligada=False, devolve_primitivas=True),
]


# Devolve uma CÓPIA da lista com algumas etapas desligadas. A lista
# PIPELINE_PADRAO é compartilhada por quem importa este módulo; se a gente
# fizesse etapa.ligada = False direto nela, o próximo a usar herdaria a
# mudança. replace() cria uma Etapa nova igual, só com o campo trocado.
def sem(etapas, *nomes):
    existentes = {e.nome for e in etapas}
    desconhecidas = set(nomes) - existentes
    if desconhecidas:
        raise ValueError(f"etapa(s) desconhecida(s): {sorted(desconhecidas)}; "
                         f"as etapas são: {[e.nome for e in etapas]}")
    return [replace(e, ligada=False) if e.nome in nomes else e for e in etapas]


# ---------------------------------------------------------------------------
# 3. Rodar uma etapa uma vez, conferindo o que ela devolveu
#
# O NotImplementedError do ajuste vira uma mensagem que diz o que fazer.
# Ele aparece já no aquecimento, antes de qualquer medida.
# ---------------------------------------------------------------------------
def _executar(etapa, nuvem):
    try:
        saida = etapa.funcao(nuvem, **etapa.parametros)
    except NotImplementedError as erro:
        raise RuntimeError(
            f"a etapa '{etapa.nome}' está ligada, mas a função "
            f"{etapa.funcao.__module__}.{etapa.funcao.__name__}() ainda não "
            f"foi implementada ({erro}). Desligue a etapa (ligada=False, ou "
            f"--sem {etapa.nome} no scripts/rodar_pipeline.py).") from erro

    if not etapa.devolve_primitivas and not isinstance(saida, Nuvem):
        raise TypeError(
            f"a etapa '{etapa.nome}' devolveu {type(saida).__name__}, mas um "
            f"filtro tem que devolver uma Nuvem (regra 2 do combinado em "
            f"percepcao/nuvem.py). Se ela é o ajuste de primitivas, marque "
            f"devolve_primitivas=True.")
    return saida


# ---------------------------------------------------------------------------
# 4. Rodar o pipeline, medindo cada etapa
#
# Para cada etapa ligada:
#   - 1 execução de AQUECIMENTO, descartada. A primeira chamada costuma ser
#     mais lenta (o Open3D e o numpy carregam coisas, a memória ainda não
#     foi reservada) e puxaria a média para cima.
#   - "repeticoes" execuções medidas com time.perf_counter(), o relógio mais
#     preciso do Python para intervalos curtos. Só a chamada da função fica
#     dentro do cronômetro: contar pontos, avaliar e visualizar ficam fora.
#     A função cronometrada é a funcao_tempo, se a etapa tiver uma, ou a
#     própria funcao (seção 2).
#   - a saída vira a entrada da próxima etapa. Sem funcao_tempo, é a saída
#     da última execução medida (as etapas são determinísticas, então todas
#     dão o mesmo resultado). Com funcao_tempo, a funcao roda uma vez a
#     mais, fora do cronômetro, só para produzir a saída com rótulos.
#
# Devolve (nuvem_final, primitivas, relatorio):
#   primitivas: a lista do ajuste, ou None se essa etapa estava desligada
#   relatorio:  uma linha (dicionário) por etapa, na ordem da lista:
#       etapa, ligada, tempo_ms (média), desvio_ms (desvio-padrão),
#       tempo_de ("funcao" ou "funcao_tempo": qual função foi cronometrada),
#       pontos_entrada, pontos_saida, por_objeto (id -> pontos na saída)
#   Etapa desligada: ligada=False e o resto None.
#
# O desvio-padrão é o amostral (ddof=1, divide por n-1), o usual quando as
# medidas são uma amostra. Com uma repetição só, não há desvio: fica 0.
# ---------------------------------------------------------------------------
def rodar(nuvem, etapas, repeticoes=5):
    if repeticoes < 1:
        raise ValueError("repeticoes tem que ser pelo menos 1")

    primitivas = None
    relatorio = []

    for etapa in etapas:
        if not etapa.ligada:
            relatorio.append({"etapa": etapa.nome, "ligada": False,
                              "tempo_ms": None, "desvio_ms": None,
                              "tempo_de": None,
                              "pontos_entrada": None, "pontos_saida": None,
                              "por_objeto": None})
            continue

        if etapa.funcao_tempo is None:
            _executar(etapa, nuvem)            # aquecimento, descartado
            tempos = []
            for _ in range(repeticoes):
                t0 = time.perf_counter()
                saida = _executar(etapa, nuvem)
                tempos.append(time.perf_counter() - t0)
        else:
            # Primeiro a saída de verdade (com rótulos), fora do tempo. Ela
            # vem antes para um erro da funcao aparecer logo, como nas outras.
            saida = _executar(etapa, nuvem)
            etapa.funcao_tempo(nuvem, **etapa.parametros)   # aquecimento
            tempos = []
            for _ in range(repeticoes):
                t0 = time.perf_counter()
                etapa.funcao_tempo(nuvem, **etapa.parametros)
                tempos.append(time.perf_counter() - t0)
        tempos_ms = 1000 * np.array(tempos)

        entrada = len(nuvem)
        if etapa.devolve_primitivas:
            # O ajuste não muda a nuvem: ela segue igual, e as primitivas
            # vão à parte.
            primitivas = saida
        else:
            nuvem = saida

        relatorio.append({
            "etapa": etapa.nome,
            "ligada": True,
            "tempo_ms": float(tempos_ms.mean()),
            "desvio_ms": float(tempos_ms.std(ddof=1)) if repeticoes > 1 else 0.0,
            "tempo_de": "funcao" if etapa.funcao_tempo is None else "funcao_tempo",
            "pontos_entrada": entrada,
            "pontos_saida": len(nuvem),
            "por_objeto": nuvem.contagem(),
        })

    return nuvem, primitivas, relatorio
