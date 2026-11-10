"""
percepcao: do que o sensor vê até o modelo leve do ambiente.

Duas metades:
  simulacao/  gera o que o "olho" (a câmera de profundidade) vê: cenas do
              PyBullet, câmera, projeção para nuvem de pontos e a trajetória
              da aranha
  etapas/     o "cérebro": filtros, normais e (depois) ajuste de primitivas

No meio, o formato comum (nuvem.py: a classe Nuvem) e, do lado, a
comparação com o ground truth (avaliacao.py).
"""
