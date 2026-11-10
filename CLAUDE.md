# projectMike

Projeto de pesquisa. Um robô que anda sobre tubulações
precisa de um modelo leve do ambiente: transformamos nuvens
de pontos (simuladas no PyBullet) em primitivas geométricas (cilindros e planos)
e comparamos com o ground truth da simulação.

- Pacote `percepcao/` (Python 3.10–3.12, venv em `.venv` na raiz, instalado com
  `pip install -e ".[dev]"`). Scripts em `scripts/`, testes em `tests/` (`pytest`).
  Os caminhos de `data/` partem da raiz, então os scripts rodam de qualquer pasta.
- Etapas (temática Michelangelo): Carrara (cenas) → Marmo (nuvem) → Scalpello
  (filtros) → Michelangelo (RANSAC: CGAL Efficient RANSAC; pyRANSAC-3D como
  baseline) → David (primitivas) → Vasari (métricas e relatórios).
  Carrara e Marmo ficam em `percepcao/simulacao/`, Scalpello e Michelangelo em
  `percepcao/etapas/`, David em `etapas/primitivas.py`, Vasari em `avaliacao.py`.
- Código e comentários em português, no estilo dos arquivos existentes.
- Não commitar PDFs/artigos/planos, nem os quadros gerados em `data/galho/`.

## Regra principal

Este é um projeto de aprendizado. **Pode implementar quando a gente pedir;
explique o que fez em português didático e pare nas etapas combinadas.**
Sem pedido explícito, explique e revise: aponte erros, explique conceitos e
sugira caminhos.
