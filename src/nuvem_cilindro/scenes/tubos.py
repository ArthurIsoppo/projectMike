"""
Cena de teste: um piso e 3 cilindros com medidas conhecidas.

Rodar (de dentro da pasta nuvem_cilindro, com o venv ativo):
    python scenes/tubos.py

Abre a janela do PyBullet com a cena e salva o ground truth em data/gt.json.
"""
import json
import math
from pathlib import Path

import pybullet as p
import pybullet_data

# ---------------------------------------------------------------------------
# 1. Parâmetros dos cilindros (tudo em metros e radianos)
#
#   raio, altura: tamanho do cilindro
#   centro:       posição do CENTRO do cilindro no mundo (não da base)
#   euler:        rotação [em X, em Y, em Z]. [0, 0, 0] = cilindro em pé,
#                 com o eixo apontando para cima (eixo Z do mundo)
#   cor:          [vermelho, verde, azul, opacidade], de 0 a 1
# ---------------------------------------------------------------------------
CILINDROS = [
    {   # pilar grosso, em pé: centro em z = altura/2 para encostar no chão
        "nome": "pilar",
        "raio": 0.15, "altura": 2.0,
        "centro": [0.0, 0.0, 1.0],
        "euler": [0.0, 0.0, 0.0],
        "cor": [0.8, 0.2, 0.2, 1.0],
    },
    {   # tubo médio deitado no chão: girado 90° em Y, então o eixo fica
        # ao longo de X. Deitado, o centro fica em z = raio
        "nome": "tubo_piso",
        "raio": 0.08, "altura": 1.5,
        "centro": [1.0, 0.8, 0.08],
        "euler": [0.0, math.pi / 2, 0.0],
        "cor": [0.2, 0.6, 0.2, 1.0],
    },
    {   # tubo fino inclinado, suspenso no ar: 45° em X e 30° em Z
        "nome": "tubo_elevado",
        "raio": 0.05, "altura": 1.8,
        "centro": [0.8, -0.8, 1.2],
        "euler": [math.pi / 4, 0.0, math.pi / 6],
        "cor": [0.2, 0.3, 0.8, 1.0],
    },
]

# Pasta data/ ao lado de scenes/, funcione de onde o script for chamado
PASTA_DATA = Path(__file__).resolve().parent.parent / "data"


# ---------------------------------------------------------------------------
# 2a. Malha de um cilindro com muitas faces
#
# Devolve os vértices (pontos), os índices (cada 3 índices = 1 triângulo)
# e as normais de cada vértice. O cilindro fica em pé, com o eixo no Z e a
# origem no centro, igual ao GEOM_CYLINDER.
# ---------------------------------------------------------------------------
def malha_cilindro(raio, altura, n_faces=128):
    vertices, normais, indices = [], [], []
    meia = altura / 2.0

    # Lateral: para cada ângulo, um vértice embaixo e um em cima.
    # A normal aponta para fora do eixo (cos, sin, 0)
    for i in range(n_faces):
        a = 2.0 * math.pi * i / n_faces
        c, s = math.cos(a), math.sin(a)
        vertices += [[raio * c, raio * s, -meia], [raio * c, raio * s, meia]]
        normais += [[c, s, 0.0], [c, s, 0.0]]
    for i in range(n_faces):
        j = (i + 1) % n_faces                 # próximo ângulo (fecha a volta)
        b0, t0, b1, t1 = 2 * i, 2 * i + 1, 2 * j, 2 * j + 1
        indices += [b0, b1, t1, b0, t1, t0]   # 2 triângulos por face

    # Tampas: um vértice no centro e um anel em volta, em cada ponta
    for z, nz in [(meia, 1.0), (-meia, -1.0)]:
        centro = len(vertices)
        vertices.append([0.0, 0.0, z])
        normais.append([0.0, 0.0, nz])
        for i in range(n_faces):
            a = 2.0 * math.pi * i / n_faces
            vertices.append([raio * math.cos(a), raio * math.sin(a), z])
            normais.append([0.0, 0.0, nz])
        for i in range(n_faces):
            k0, k1 = centro + 1 + i, centro + 1 + (i + 1) % n_faces
            # a ordem dos vértices define para que lado o triângulo "olha"
            indices += [centro, k0, k1] if nz > 0 else [centro, k1, k0]

    return vertices, indices, normais


# ---------------------------------------------------------------------------
# 2b. Função que cria UM cilindro na cena e devolve o id dele e o eixo
# ---------------------------------------------------------------------------
def criar_cilindro(raio, altura, centro, euler, cor):
    # O PyBullet não usa ângulos de Euler direto, usa quaternion
    orientacao = p.getQuaternionFromEuler(euler)

    # Forma de colisão (física) e forma visual (o que a câmera enxerga).
    # Pegadinha da API: na colisão a altura chama "height", no visual "length"
    colisao = p.createCollisionShape(p.GEOM_CYLINDER, radius=raio, height=altura)
    # O visual NÃO usa o GEOM_CYLINDER: o do PyBullet é um prisma de poucas
    # faces, e os pontos da câmera ficariam até 7% para dentro do raio real.
    # Usamos uma malha própria com 128 faces (ver malha_cilindro acima)
    vertices, indices, normais = malha_cilindro(raio, altura)
    visual = p.createVisualShape(p.GEOM_MESH, vertices=vertices,
                                 indices=indices, normals=normais,
                                 rgbaColor=cor)

    # baseMass=0 deixa o objeto estático: a gravidade não mexe nele,
    # então a pose fica igual à do ground truth
    corpo = p.createMultiBody(baseMass=0,
                              baseCollisionShapeIndex=colisao,
                              baseVisualShapeIndex=visual,
                              basePosition=centro,
                              baseOrientation=orientacao)

    # Eixo do cilindro no mundo = eixo Z local girado pela orientação.
    # getMatrixFromQuaternion devolve a matriz de rotação 3x3 como uma
    # lista de 9 números (linha por linha); a 3ª coluna são os índices 2, 5, 8
    R = p.getMatrixFromQuaternion(orientacao)
    eixo = [R[2], R[5], R[8]]

    return corpo, eixo


# ---------------------------------------------------------------------------
# 3. Monta a cena inteira e devolve o ground truth
#    (separado do "main" para os próximos scripts poderem reaproveitar)
# ---------------------------------------------------------------------------
def montar_cena():
    p.setAdditionalSearchPath(pybullet_data.getDataPath())  # onde fica o plane.urdf
    p.setGravity(0, 0, -9.81)
    id_piso = p.loadURDF("plane.urdf")

    gt = {"unidade": "m", "id_piso": id_piso, "cilindros": []}

    for c in CILINDROS:
        corpo, eixo = criar_cilindro(c["raio"], c["altura"], c["centro"],
                                     c["euler"], c["cor"])
        gt["cilindros"].append({
            "nome": c["nome"],
            "id_pybullet": corpo,   # vai servir para a máscara de segmentação
            "raio": c["raio"],
            "altura": c["altura"],
            "centro": c["centro"],
            "eixo": eixo,
        })

    return gt


def salvar_gt(gt, caminho):
    caminho.parent.mkdir(parents=True, exist_ok=True)
    with open(caminho, "w") as f:
        json.dump(gt, f, indent=2)


# ---------------------------------------------------------------------------
# 4. Só roda isto quando você executa o arquivo direto
#    (não roda quando outro script faz "import" deste)
# ---------------------------------------------------------------------------
if __name__ == "__main__":
    p.connect(p.GUI)
    gt = montar_cena()
    salvar_gt(gt, PASTA_DATA / "gt.json")
    print(f"Ground truth salvo em {PASTA_DATA / 'gt.json'}")

    # Mantém a janela aberta até você fechar.
    # Como os cilindros são estáticos, não precisa simular a física.
    import time
    while p.isConnected():
        time.sleep(0.1)
