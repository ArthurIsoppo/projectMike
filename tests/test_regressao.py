"""
Testes de regressão: os números de referência, medidos ANTES da
reorganização em pacote, têm que continuar saindo iguais.

Rodar (com o venv ativo, da raiz do projeto):
    pytest

Não precisa de data/: cada teste gera as nuvens na hora (com o PyBullet em
modo DIRECT, sem janela) e grava só em pastas temporárias. Leva uns 15 s.

Tolerância: as contagens de pontos têm que bater exatamente; as medidas em
mm e em graus podem variar só na última casa decimal mostrada.
"""
import re

import pybullet as p
import pytest

from percepcao.avaliacao import conferir, medir_quadro, resumo_normais
from percepcao.etapas.filtros import reduzir_voxel
from percepcao.etapas.normais import estimar_normais
from percepcao.nuvem import Nuvem
from percepcao.simulacao import cenas, trajetoria
from percepcao.simulacao.projecao import fotografar


# Uma conexão com o PyBullet para o arquivo todo. Cada teste monta a sua
# cena do zero (resetSimulation), então a ordem dos testes não importa.
@pytest.fixture(scope="module", autouse=True)
def pybullet_direct():
    p.connect(p.DIRECT)
    yield
    p.disconnect()


def montar(nome):
    p.resetSimulation()
    return cenas.montar_tubos() if nome == "tubos" else cenas.montar_galho()


def galho_do(gt):
    return [pr for pr in gt["primitivas"] if pr["tipo"] == "cilindro"][0]


# ---------------------------------------------------------------------------
# gerar_teste.py: quantos pontos de cada objeto em cada nuvem do conjunto fixo
# ---------------------------------------------------------------------------
@pytest.mark.parametrize("n_vistas, total, contagem", [
    (1, 168402, {0: 151289, 1: 8686, 2: 3248, 3: 5179}),
    (3, 506976, {0: 445036, 1: 38138, 2: 12245, 3: 11557}),
])
def test_conjunto_tubos(n_vistas, total, contagem):
    montar("tubos")
    nv = fotografar(cenas.VISTAS_TUBOS[:n_vistas])
    assert len(nv) == total
    assert nv.contagem() == contagem


@pytest.mark.parametrize("altura, total, contagem", [
    (0.05, 238414, None),        # a referência só tem o total
    (0.15, 152868, {0: 29592, 1: 123276}),
    (0.30, 111324, {0: 38380, 1: 72944}),
])
def test_conjunto_galho(altura, total, contagem):
    cil = galho_do(montar("galho"))
    olho, alvo, cima = trajetoria.pose_aranha(cil, 2.0, altura=altura,
                                              frente=altura)
    nv = fotografar([(olho, alvo, cima)], near=trajetoria.NEAR,
                    prof_max=trajetoria.PROF_MAX)
    assert len(nv) == total
    if contagem is not None:
        assert nv.contagem() == contagem


# ---------------------------------------------------------------------------
# gerar_nuvem.py 3: a conferência contra o GT
#
# O conferir() só imprime, então lemos os números do texto impresso
# (capsys guarda o que foi para o terminal).
# ---------------------------------------------------------------------------
def test_conferencia_3_vistas(capsys):
    gt = montar("tubos")
    nv = fotografar(cenas.VISTAS_TUBOS[:3])
    assert len(nv) == 506976

    conferir(nv.pontos, nv.rotulos, gt)
    saida = capsys.readouterr().out

    m = re.search(r"piso:.*\|z\| máximo = ([\d.]+) mm", saida)
    assert m, "o piso não apareceu na conferência"
    assert float(m.group(1)) <= 0.01

    esperado = {"pilar": (150.0, 360), "tubo_piso": (80.0, 290),
                "tubo_elevado": (50.0, 360)}
    for nome, (raio, cobertura) in esperado.items():
        m = re.search(rf"{nome}\s.*medido ([\d.]+) mm \(erro máx ([\d.]+) mm\), "
                      rf"cobertura (\d+)°", saida)
        assert m, f"{nome} não apareceu na conferência"
        assert float(m.group(1)) == pytest.approx(raio, abs=0.1)
        assert float(m.group(2)) <= 0.1
        assert int(m.group(3)) == cobertura


# ---------------------------------------------------------------------------
# aranha.py: o primeiro quadro da subida
# ---------------------------------------------------------------------------
def test_quadro_0_da_aranha():
    cil = galho_do(montar("galho"))
    olho, alvo, cima = trajetoria.pose_aranha(cil, trajetoria.INICIO)
    nv = fotografar([(olho, alvo, cima)], near=trajetoria.NEAR,
                    prof_max=trajetoria.PROF_MAX)
    assert len(nv) == 200163

    m = medir_quadro(nv.pontos, nv.rotulos, cil, cima)
    assert m is not None
    arco = m["arco_graus"]
    assert isinstance(arco, float)
    assert round(arco) == 135
    assert m["raio_mediano_mm"] == pytest.approx(100.0, abs=0.1)


# ---------------------------------------------------------------------------
# exp_voxel_normais.py: voxel de 1 cm + normais com raio de 3 cm, e um ponto
# da grade (voxel 3 cm x raio 2 cm)
#
# Passa pelo .ply (salvar e carregar), como o experimento faz com o
# data/teste/tubos_3vistas.ply.
# ---------------------------------------------------------------------------
@pytest.fixture(scope="module")
def tubos_3vistas(tmp_path_factory):
    gt = montar("tubos")
    caminho = tmp_path_factory.mktemp("teste") / "tubos_3vistas.ply"
    fotografar(cenas.VISTAS_TUBOS[:3]).salvar(caminho)
    return Nuvem.carregar(caminho), gt


def test_voxel_e_normais(tubos_3vistas):
    nuvem, gt = tubos_3vistas
    red = reduzir_voxel(nuvem, tamanho=0.01)
    assert len(red) == 219384

    res = resumo_normais(estimar_normais(red, raio=0.03), gt)
    esperado = {"pilar": 0.47, "tubo_piso": 0.89, "tubo_elevado": 1.37}
    for nome, erro in esperado.items():
        assert res[nome]["erro_mediano"] == pytest.approx(erro, abs=0.01)
    # A referência "99.7%" é o valor arredondado da tabela: o código antigo
    # dá 99.68 no tubo_piso. Por isso compara com 1 casa, como é impresso.
    for nome in ["tubo_piso", "tubo_elevado"]:
        assert round(res[nome]["sentido_certo_%"], 1) >= 99.7


def test_grade_voxel3_raio2(tubos_3vistas):
    nuvem, gt = tubos_3vistas
    red = reduzir_voxel(nuvem, tamanho=0.03)
    res = resumo_normais(estimar_normais(red, raio=0.02), gt)
    assert res["tubo_elevado"]["erro_mediano"] == pytest.approx(55.75, abs=0.01)
