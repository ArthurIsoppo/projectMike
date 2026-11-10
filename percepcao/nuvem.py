"""
O formato comum do pipeline (a "interface"): o combinado que todas as etapas
seguem, para elas poderem ser trocadas, ligadas e desligadas sem reescrever
nada.

O combinado tem 3 regras:

  1. Toda nuvem é um objeto Nuvem (definido abaixo). Além dos pontos, ela
     carrega os RÓTULOS: o id do objeto de cada ponto, o ground truth por
     ponto. Eles viajam junto por todas as etapas, para dar para medir o que
     cada etapa fez com cada objeto.

  2. Toda etapa de filtro é uma função assim:
         def nome_da_etapa(nuvem: Nuvem, parametro=valor, ...) -> Nuvem
     Recebe uma Nuvem, devolve uma Nuvem nova (não altera a de entrada).
     As etapas ficam em percepcao/etapas/.

  3. A etapa final (ajuste de primitivas) devolve uma LISTA de primitivas,
     cada uma um dicionário no MESMO formato do GT:
         {"tipo": "cilindro", "centro": [x, y, z], "eixo": [ex, ey, ez],
          "raio": r, "altura": h}
         {"tipo": "plano", "normal": [nx, ny, nz], "d": d}
             (o plano é o conjunto de pontos x com  normal · x + d = 0)
     Tudo em metros, no referencial do mundo; eixo e normal com tamanho 1.
     Assim a mesma função de erro compara estimativa e GT.

Para ver uma nuvem passar pelas etapas, rode scripts/ver_etapas.py.
"""
import json
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import open3d as o3d


# ---------------------------------------------------------------------------
# 1. A Nuvem
#
#   pontos:  N x 3, em metros, no referencial do mundo
#   rotulos: N números, o id_pybullet do objeto de cada ponto (o GT por
#            ponto). -1 = ponto sem objeto (por exemplo, um outlier que a
#            gente mesmo inventou)
#   normais: N x 3 ou None (ainda não calculadas)
#   cores:   N x 3, de 0 a 1, ou None
#   origens: N x 3 ou None. De onde cada ponto foi visto: a posição da
#            câmera que o enxergou. Serve para orientar as normais (uma
#            superfície que a câmera vê está virada PARA a câmera). Num robô
#            de verdade isso também se sabe: é a pose do sensor.
#
# @dataclass é um atalho do Python: ele cria sozinho o __init__, então dá
# para fazer Nuvem(pontos=..., rotulos=...).
# ---------------------------------------------------------------------------
@dataclass
class Nuvem:
    pontos: np.ndarray
    rotulos: np.ndarray
    normais: np.ndarray | None = None
    cores: np.ndarray | None = None
    origens: np.ndarray | None = None

    def __len__(self):
        return len(self.pontos)

    # Fica só com alguns pontos, mantendo TUDO alinhado (o ponto i continua
    # com o rótulo i, a normal i...). É o que todo filtro que só JOGA FORA
    # pontos deve usar.
    def selecionar(self, indices):
        indices = np.asarray(indices)

        def pega(a):
            return None if a is None else a[indices]

        return Nuvem(pontos=self.pontos[indices], rotulos=self.rotulos[indices],
                     normais=pega(self.normais), cores=pega(self.cores),
                     origens=pega(self.origens))

    # Converte para o formato do Open3D, para usar as funções prontas dele
    # e para visualizar. Rótulos e origens ficam de fora (o Open3D não os
    # conhece), por isso as etapas trabalham com índices.
    def para_o3d(self):
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(self.pontos)
        if self.normais is not None:
            pcd.normals = o3d.utility.Vector3dVector(self.normais)
        if self.cores is not None:
            pcd.colors = o3d.utility.Vector3dVector(self.cores)
        return pcd

    # Quantos pontos de cada objeto: {0: 151289, 1: 8686, ...}
    def contagem(self):
        ids, n = np.unique(self.rotulos, return_counts=True)
        return dict(zip(ids.tolist(), n.tolist()))

    # Ler e gravar. O .ply guarda pontos, cores e normais; rótulos e origens
    # vão em arquivos .npy ao lado (o .ply do Open3D não guarda campos extras)
    @staticmethod
    def carregar(caminho_ply):
        caminho_ply = Path(caminho_ply)
        pcd = o3d.io.read_point_cloud(str(caminho_ply))
        arq_origens = _caminho_extra(caminho_ply, "origens")
        return Nuvem(
            pontos=np.asarray(pcd.points),
            rotulos=np.load(_caminho_extra(caminho_ply, "rotulos")),
            normais=np.asarray(pcd.normals) if pcd.has_normals() else None,
            cores=np.asarray(pcd.colors) if pcd.has_colors() else None,
            origens=np.load(arq_origens) if arq_origens.exists() else None,
        )

    def salvar(self, caminho_ply):
        caminho_ply = Path(caminho_ply)
        caminho_ply.parent.mkdir(parents=True, exist_ok=True)
        o3d.io.write_point_cloud(str(caminho_ply), self.para_o3d())
        np.save(_caminho_extra(caminho_ply, "rotulos"), self.rotulos)
        if self.origens is not None:
            np.save(_caminho_extra(caminho_ply, "origens"), self.origens)


# Nome dos arquivos extras ao lado do .ply:
#   tubos_3vistas.ply -> tubos_3vistas_rotulos.npy, tubos_3vistas_origens.npy
#   nuvem.ply         -> rotulos.npy, origens.npy (como o gerar_nuvem.py grava)
def _caminho_extra(caminho_ply, tipo):
    if caminho_ply.stem == "nuvem":
        return caminho_ply.with_name(f"{tipo}.npy")
    return caminho_ply.with_name(f"{caminho_ply.stem}_{tipo}.npy")


def carregar_gt(caminho_json):
    with open(caminho_json) as f:
        return json.load(f)
