"""
Filtros: etapas que recebem uma Nuvem e devolvem uma Nuvem com menos pontos
(regra 2 do combinado em percepcao/nuvem.py).
"""
import numpy as np

from percepcao.nuvem import Nuvem


# Remove outliers estatísticos: para cada ponto, olha a distância média até
# os "vizinhos" mais próximos; descarta quem está bem mais longe que o normal
# (mais de "desvios" desvios-padrão acima da média).
def remover_outliers(nuvem, vizinhos=20, desvios=2.0):
    _, mantidos = nuvem.para_o3d().remove_statistical_outlier(
        nb_neighbors=vizinhos, std_ratio=desvios)
    return nuvem.selecionar(mantidos)


# Voxel grid: divide o espaço em cubinhos de lado "tamanho" (metros) e troca
# todos os pontos de cada cubinho por um só (a média deles).
#
# O ponto novo não existia antes, então qual rótulo ele recebe? O mais comum
# entre os pontos que caíram naquele cubinho. Por isso usamos a versão
# "_and_trace", que diz quais pontos originais foram para cada cubinho.
# A origem (de onde foi visto) é a do primeiro ponto do cubinho: qualquer
# câmera que viu aquele pedaço de superfície serve para orientar a normal.
def reduzir_voxel(nuvem, tamanho=0.01):
    pcd = nuvem.para_o3d()
    reduzida, _, origem = pcd.voxel_down_sample_and_trace(
        tamanho, pcd.get_min_bound(), pcd.get_max_bound(), False)

    # Para cada cubinho v, a lista de pontos originais que caíram nele.
    # Juntamos tudo num vetor só (indices) e anotamos de qual cubinho cada
    # um veio (cubinho), para fazer as contas sem um "for" lento.
    tamanhos = np.array([len(i) for i in origem])
    indices = np.concatenate([np.asarray(i) for i in origem])
    cubinho = np.repeat(np.arange(len(origem)), tamanhos)

    # Rótulo mais comum por cubinho: monta uma tabela cubinho x rótulo com
    # as contagens e pega a coluna com o maior número em cada linha.
    # (+1 porque o rótulo -1 viraria um índice negativo)
    rot = nuvem.rotulos[indices] + 1
    n_rot = rot.max() + 1
    tabela = np.bincount(cubinho * n_rot + rot,
                         minlength=len(origem) * n_rot).reshape(-1, n_rot)
    rotulos = tabela.argmax(axis=1) - 1

    primeiro = indices[np.concatenate([[0], np.cumsum(tamanhos)[:-1]])]
    return Nuvem(
        pontos=np.asarray(reduzida.points),
        rotulos=rotulos,
        normais=None,      # normais antigas não valem mais: recalcular depois
        cores=np.asarray(reduzida.colors) if reduzida.has_colors() else None,
        origens=None if nuvem.origens is None else nuvem.origens[primeiro],
    )
