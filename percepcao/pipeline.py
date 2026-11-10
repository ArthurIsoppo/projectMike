"""
Pipeline integrado. AINDA NÃO IMPLEMENTADO.

Vai encadear as etapas, sempre de Nuvem em Nuvem (ver percepcao/nuvem.py):

    remover_outliers -> reduzir_voxel -> estimar_normais -> ajuste de primitivas
    (etapas/filtros)    (etapas/filtros)  (etapas/normais)  (etapas/primitivas)

e devolver a lista de primitivas, para comparar com o GT.
"""
