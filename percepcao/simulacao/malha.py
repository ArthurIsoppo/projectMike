"""
Cilindros na cena do PyBullet: a malha com muitas faces e a função que cria
um cilindro estático e devolve o eixo dele no mundo.
"""
import math

import pybullet as p


# ---------------------------------------------------------------------------
# 1. Malha de um cilindro com muitas faces
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
# 2. Função que cria UM cilindro na cena e devolve o id dele e o eixo
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
