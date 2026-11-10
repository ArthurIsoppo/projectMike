"""
A trajetória da aranha: uma câmera que escala o galho, andando ao longo do
eixo do tubo, um pouco acima da superfície.

Para rodar a aranha e gerar os quadros, use scripts/aranha.py.
"""
import numpy as np

# ---------------------------------------------------------------------------
# 1. Parâmetros da aranha
#
#   ALTURA:   distância da câmera até a superfície do tubo (metros).
#             O plano pede para testar algumas alturas: é aqui que muda.
#   FRENTE:   quão à frente, ao longo do tubo, a câmera mira (metros).
#             Maior = olha mais "para a frente"; menor = olha mais para baixo.
#             Com FRENTE = ALTURA, a câmera olha 45° para baixo e vê só um
#             trecho curto do tubo (o caso difícil que o plano pede).
#   INICIO, FIM: trecho do tubo que a aranha percorre, medido a partir da base.
#   N_QUADROS:   quantas fotos tira no caminho.
#   PROF_MAX:    descarta pontos a mais de X metros da câmera.
#   NEAR:        a câmera está muito perto do tubo, então o "near" de 10 cm
#                do camera.py cortaria a superfície. Usamos 2 cm.
# ---------------------------------------------------------------------------
ALTURA = 0.15
FRENTE = 0.15
INICIO, FIM = 0.3, 5.0
N_QUADROS = 20
PROF_MAX = 3.0
NEAR = 0.02


# ---------------------------------------------------------------------------
# 2. Onde a aranha está e para onde olha, a uma distância s da base
#
# "cil" é a primitiva do galho no GT (um dicionário com "tipo": "cilindro").
#
# Três vetores definem a pose:
#   eixo: direção do tubo (para onde a aranha anda)
#   cima: o "para cima" da aranha. É perpendicular ao eixo e aponta para o
#         lado de cima do tubo. Sai do Z do mundo, tirando a parte que está
#         na direção do eixo (a mesma conta da "distância ao eixo").
#   olho: ponto no eixo + (raio + ALTURA) na direção "cima"
#   alvo: um ponto da superfície, FRENTE metros adiante
#
# Por que não usar o Z do mundo como "cima" da câmera, como no camera.py?
# Se o tubo fosse vertical, a câmera olharia na mesma direção do Z, e o
# "cima" ficaria indefinido (a foto poderia sair girada de qualquer jeito).
# Usando o "cima" do próprio tubo, funciona em qualquer inclinação.
# ---------------------------------------------------------------------------
def pose_aranha(cil, s, altura=ALTURA, frente=FRENTE):
    eixo = np.array(cil["eixo"])
    base = np.array(cil["centro"]) - eixo * cil["altura"] / 2

    z_mundo = np.array([0.0, 0.0, 1.0])
    if abs(z_mundo @ eixo) > 0.99:            # tubo quase vertical
        z_mundo = np.array([1.0, 0.0, 0.0])
    cima = z_mundo - (z_mundo @ eixo) * eixo
    cima /= np.linalg.norm(cima)

    olho = base + s * eixo + (cil["raio"] + altura) * cima
    alvo = base + (s + frente) * eixo + cil["raio"] * cima
    return olho, alvo, cima
