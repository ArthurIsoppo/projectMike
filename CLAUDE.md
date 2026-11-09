# projectMike

Projeto de pesquisa. Um robô que anda sobre tubulações
precisa de um modelo leve do ambiente: transformamos nuvens
de pontos (simuladas no PyBullet) em primitivas geométricas (cilindros e planos)
e comparamos com o ground truth da simulação.

- Código em `src/nuvem_cilindro/` (Python 3.12, venv em `src/nuvem_cilindro/.venv`).
  Rode os scripts de dentro dessa pasta.
- Etapas (temática Michelangelo): Carrara (cenas) → Marmo (nuvem) → Scalpello
  (filtros) → Michelangelo (RANSAC: CGAL Efficient RANSAC; pyRANSAC-3D como
  baseline) → David (primitivas) → Vasari (métricas e relatórios).
- Código e comentários em português, no estilo dos arquivos existentes.
- Não commitar PDFs/artigos/planos, nem os quadros gerados em `data/galho/`.

## Regra principal

Este é um projeto de aprendizado. **Explique e revise o código; não escreva a
solução por nós.** Aponte erros, explique conceitos, sugira caminhos e mostre
trechos curtos de exemplo quando ajudar, mas a implementação é nossa.
