"""
Normais: etapa que devolve a mesma Nuvem com as normais preenchidas
(regra 2 do combinado em percepcao/nuvem.py).
"""
from dataclasses import replace

import numpy as np
import open3d as o3d


# Normais em 2 passos:
#
#   a) Direção: para cada ponto, pega os vizinhos dentro de "raio" (no máximo
#      "max_vizinhos"), ajusta um plano neles e usa a direção perpendicular.
#      Esse plano não sabe o que é "fora" ou "dentro": a normal pode sair
#      com o sentido trocado.
#   b) Sentido: vira a normal para o lado da câmera que viu o ponto. Se
#      normal · (câmera - ponto) < 0, ela aponta para longe da câmera e é
#      invertida. Num cilindro visto de fora, isso faz a normal apontar para
#      fora do eixo; no piso, para cima.
#
# O raio é o parâmetro importante: pequeno demais pega poucos vizinhos e a
# normal fica instável; grande demais "vê" a curvatura do tubo e entorta a
# normal (pior nos tubos finos).
#
# max_vizinhos=None usa TODOS os vizinhos dentro do raio. Cuidado: se passar
# um número, o Open3D reserva memória para esse número em cada ponto, e fica
# lento mesmo quando acha poucos (500 deixou tudo 20x mais lento).
def estimar_normais(nuvem, raio=0.03, max_vizinhos=None):
    pcd = nuvem.para_o3d()
    if max_vizinhos is None:
        busca = o3d.geometry.KDTreeSearchParamRadius(raio)
    else:
        busca = o3d.geometry.KDTreeSearchParamHybrid(radius=raio,
                                                     max_nn=max_vizinhos)
    pcd.estimate_normals(busca)
    normais = np.asarray(pcd.normals).copy()

    if nuvem.origens is not None:
        para_camera = nuvem.origens - nuvem.pontos
        trocar = np.einsum("ij,ij->i", normais, para_camera) < 0   # produto escalar linha a linha
        normais[trocar] *= -1

    return replace(nuvem, normais=normais)
