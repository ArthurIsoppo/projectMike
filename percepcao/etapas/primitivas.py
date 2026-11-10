"""
Ajuste de primitivas: a etapa final do pipeline. AINDA NÃO IMPLEMENTADA.

Contrato (regra 3 do combinado em percepcao/nuvem.py):

  Entrada: uma Nuvem (de preferência já filtrada e com normais).
  Saída:   uma LISTA de primitivas, cada uma um dicionário no MESMO formato
           das "primitivas" do GT (ver percepcao/simulacao/cenas.py):

      {"tipo": "cilindro", "centro": [x, y, z], "eixo": [ex, ey, ez],
       "raio": r, "altura": h}
      {"tipo": "plano", "normal": [nx, ny, nz], "d": d}
          (o plano é o conjunto de pontos x com  normal · x + d = 0)

  Tudo em metros, no referencial do mundo; eixo e normal com tamanho 1.
  Assim a mesma função de erro (em percepcao/avaliacao.py) compara
  estimativa e GT.
"""


def ajustar(nuvem):
    raise NotImplementedError("o ajuste de primitivas ainda não foi implementado")
