"""
Avaliação contra o ground truth: tudo que compara o que o pipeline produziu
com a "verdade" da cena simulada.

  conferir:       a projeção está certa? (piso em z = 0, cilindros no raio)
  medir_quadro:   o que a aranha viu do galho em um quadro
  normais:        normais_verdadeiras, erro_normais, resumo_normais

Depois entra aqui também o erro das primitivas (raio e eixo), usando o
mesmo GT.
"""
import numpy as np


# ---------------------------------------------------------------------------
# 1. Conferência da projeção: os pontos de cada cilindro estão mesmo a
#    "raio" do eixo?
#
# Para cada ponto, calculamos a distância até a reta do eixo do GT.
# Se a conversão estiver certa, essa distância é igual ao raio.
# Os pontos das tampas (pontas do cilindro) ficam fora da conta, porque
# lá a distância ao eixo vai de 0 até o raio.
# O plano (o piso) é conferido pela altura: tem que estar em z = 0.
# ---------------------------------------------------------------------------
def conferir(pontos, rotulos, gt):
    for c in gt["primitivas"]:
        if c["tipo"] == "plano":
            piso = pontos[rotulos == c["id_pybullet"]]
            print(f"  piso: {len(piso):6d} pontos, |z| máximo = "
                  f"{np.abs(piso[:, 2]).max() * 1000:.2f} mm (deveria ser ~0)")
            continue

        sel = pontos[rotulos == c["id_pybullet"]]
        if len(sel) == 0:
            print(f"  {c['nome']}: nenhum ponto!")
            continue
        eixo, centro = np.array(c["eixo"]), np.array(c["centro"])
        rel = sel - centro
        t = rel @ eixo                              # posição ao longo do eixo
        lateral = np.abs(t) < c["altura"] / 2 - 0.005   # tira as tampas
        dist = np.linalg.norm(rel - np.outer(t, eixo), axis=1)[lateral]
        erro = (dist - c["raio"]) * 1000

        # Cobertura: quantos graus da volta do cilindro têm pontos.
        # Montamos dois vetores u e w perpendiculares ao eixo (um "relógio"
        # em volta do eixo), medimos o ângulo de cada ponto nesse relógio e
        # contamos quantas fatias de 10° têm pelo menos um ponto.
        ref = [1.0, 0.0, 0.0] if abs(eixo[0]) < 0.9 else [0.0, 1.0, 0.0]
        u = np.cross(eixo, ref)
        u /= np.linalg.norm(u)
        w = np.cross(eixo, u)
        radial = (rel - np.outer(t, eixo))[lateral]
        angulo = np.degrees(np.arctan2(radial @ w, radial @ u))   # -180 a 180
        fatias = np.unique(((angulo + 180) // 10).astype(int))
        cobertura = 10 * len(fatias)

        print(f"  {c['nome']:13s} {len(sel):6d} pontos, raio GT "
              f"{c['raio'] * 1000:.0f} mm, medido {np.median(dist) * 1000:.1f} mm "
              f"(erro máx {np.abs(erro).max():.1f} mm), "
              f"cobertura {cobertura}° de 360°")


# ---------------------------------------------------------------------------
# 2. O que a aranha viu do galho em um quadro
#
#   cil:    a primitiva do galho no GT; cima: o "cima" da pose da aranha
#           (ver simulacao/trajetoria.py)
#
#   arco:   quantos graus da volta do tubo aparecem. A aranha vê uma faixa
#           contínua em volta do topo, então basta o maior ângulo menos o
#           menor (o topo é 0°, um lado é negativo e o outro positivo)
#   trecho: de onde até onde, ao longo do tubo, aparecem pontos
#   teorico: o maior arco que dá para ver de um ponto a uma distância D do
#            eixo é 2 * arccos(raio / D). Com raio 0,10 e D = 0,25, dá 133°.
#            Serve para conferir se o arco medido faz sentido.
#   raio:   mediana da distância dos pontos ao eixo (deveria ser o raio)
# ---------------------------------------------------------------------------
def medir_quadro(pontos, rotulos, cil, cima):
    eixo = np.array(cil["eixo"])
    base = np.array(cil["centro"]) - eixo * cil["altura"] / 2
    sel = pontos[rotulos == cil["id_pybullet"]]
    if len(sel) == 0:
        return None

    rel = sel - base
    t = rel @ eixo                         # posição ao longo do tubo
    radial = rel - np.outer(t, eixo)       # parte perpendicular ao eixo
    dist = np.linalg.norm(radial, axis=1)

    # Ângulo de cada ponto em volta do eixo, com 0° = topo do tubo ("cima")
    lado = np.cross(eixo, cima)
    angulo = np.degrees(np.arctan2(radial @ lado, radial @ cima))

    return {
        "pontos_galho": int(len(sel)),
        "arco_graus": float(angulo.max() - angulo.min()),
        "trecho_m": [float(t.min()), float(t.max())],
        "raio_mediano_mm": float(np.median(dist) * 1000),
    }


# ---------------------------------------------------------------------------
# 3. A normal VERDADEIRA de cada ponto, calculada a partir do GT
#
#   plano:    a normal do plano, a mesma para todos os pontos dele
#   cilindro: na lateral, a direção que sai do eixo e passa pelo ponto
#             (a "parte de lado" da conta da distância ao eixo, com
#             tamanho 1). Nas tampas, o próprio eixo, para cima ou para
#             baixo conforme a tampa.
#   rótulo sem primitiva (-1): NaN (não entra nas contas)
#
# Todas apontam para FORA do objeto, que é o sentido que queremos nas
# normais estimadas.
# ---------------------------------------------------------------------------
def normais_verdadeiras(nuvem, gt):
    verdade = np.full((len(nuvem), 3), np.nan)

    for pr in gt["primitivas"]:
        sel = nuvem.rotulos == pr["id_pybullet"]
        if not sel.any():
            continue

        if pr["tipo"] == "plano":
            verdade[sel] = pr["normal"]

        elif pr["tipo"] == "cilindro":
            eixo, centro = np.array(pr["eixo"]), np.array(pr["centro"])
            rel = nuvem.pontos[sel] - centro
            t = rel @ eixo                       # posição ao longo do eixo
            lateral = rel - np.outer(t, eixo)    # parte perpendicular ao eixo
            n = lateral / np.linalg.norm(lateral, axis=1, keepdims=True)

            # Tampas: pontos perto das pontas (a menos de 2 mm do fim) e
            # mais para dentro que o raio
            tampa = (np.abs(t) > pr["altura"] / 2 - 0.002) & \
                    (np.linalg.norm(lateral, axis=1) < pr["raio"] * 0.98)
            n[tampa] = np.outer(np.sign(t[tampa]), eixo)
            verdade[sel] = n

    return verdade


# ---------------------------------------------------------------------------
# 4. Erro angular entre a normal estimada e a verdadeira, em graus
#
#   direcao:  ignora o sentido (uma normal para dentro e uma para fora na
#             mesma reta dão erro 0). Mede se a RETA está certa:
#                 arccos(|n_est · n_gt|)          vai de 0° a 90°
#   sentido:  com o sentido. Uma normal invertida dá ~180°:
#                 arccos(n_est · n_gt)            vai de 0° a 180°
#
# Separar os dois mostra se o problema está na estimativa (a direção) ou só
# na orientação (o sentido).
# ---------------------------------------------------------------------------
def erro_normais(normais_est, normais_gt):
    cos = np.einsum("ij,ij->i", normais_est, normais_gt)   # produto escalar por linha
    cos = np.clip(cos, -1.0, 1.0)
    return {
        "direcao": np.degrees(np.arccos(np.abs(cos))),
        "sentido": np.degrees(np.arccos(cos)),
    }


# Resumo por objeto: mediana e percentil 90 do erro de direção, e a
# porcentagem de normais com o sentido certo (erro com sentido < 90°)
def resumo_normais(nuvem, gt):
    verdade = normais_verdadeiras(nuvem, gt)
    erro = erro_normais(nuvem.normais, verdade)
    resumo = {}
    for pr in gt["primitivas"]:
        sel = (nuvem.rotulos == pr["id_pybullet"]) & ~np.isnan(verdade[:, 0])
        if not sel.any():
            continue
        d = erro["direcao"][sel]
        resumo[pr["nome"]] = {
            "pontos": int(sel.sum()),
            "erro_mediano": float(np.median(d)),
            "erro_p90": float(np.percentile(d, 90)),
            "sentido_certo_%": float(100 * np.mean(erro["sentido"][sel] < 90)),
        }
    return resumo
