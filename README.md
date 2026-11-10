# percepcao

Um robô que anda sobre tubulações precisa de um modelo **leve** do ambiente.
A proposta é resumir a nuvem de pontos em poucas primitivas geométricas:
tubos e pilares viram **cilindros** (eixo, raio, comprimento) e piso e paredes
viram **planos**. As cenas são simuladas no PyBullet, onde sabemos a posição e o
tamanho exatos de cada objeto, para comparar tudo com o ground truth (GT).

O pacote tem duas metades:

| Metade | Pasta | O que faz |
|---|---|---|
| o "olho" | `percepcao/simulacao/` | cenas, câmera de profundidade, projeção para nuvem, trajetória da aranha |
| o "cérebro" | `percepcao/etapas/` | outliers, voxel, normais e (depois) ajuste de primitivas |

No meio fica o formato comum (`percepcao/nuvem.py`, a classe `Nuvem`). Ao lado
fica a comparação com o GT (`percepcao/avaliacao.py`).

Na temática Michelangelo: **Carrara** (cenas) e **Marmo** (nuvem) estão em
`simulacao/`, **Scalpello** (filtros) e **Michelangelo** (RANSAC) em `etapas/`,
**David** é a lista de primitivas que `etapas/primitivas.py` vai devolver, e
**Vasari** (métricas) é o `avaliacao.py`.

## Como instalar

É preciso **Python 3.10, 3.11 ou 3.12**: o `pybullet` não compila no 3.13.

```bash
cd instor-percepcao
python3 -m venv .venv
source .venv/bin/activate
pip install -e ".[dev]"      # a compilação do pybullet leva alguns minutos
```

O `-e` instala o pacote "editável": dá para mudar o código em `percepcao/` sem
reinstalar. Por isso os imports são normais (`from percepcao.nuvem import
Nuvem`) e funcionam de qualquer pasta, sem `sys.path.insert`.

Dependências: `pybullet`, `numpy`, `open3d` e `matplotlib` (e `pytest` no
`[dev]`).

## Como rodar

Com o venv ativo, de qualquer pasta (os caminhos de `data/` partem da raiz do
projeto):

```bash
# Ver (abrem janela)
python scripts/ver_cena.py tubos      # cena dos tubos; salva data/gt.json
python scripts/ver_cena.py galho      # o galho que a aranha escala
python scripts/ver_camera.py          # RGB / profundidade / máscara; salva data/camera_vista.png

# Gerar nuvens
python scripts/gerar_nuvem.py         # 3 vistas -> data/nuvem.ply, confere contra o GT e abre o Open3D
python scripts/gerar_nuvem.py 1       # só a primeira vista
python scripts/aranha.py              # 20 quadros em data/galho/ (sem janela)
python scripts/aranha.py gui          # o mesmo, mostrando a aranha subindo
python scripts/gerar_teste.py         # congela o conjunto de teste em data/teste/

# Experimentos (precisam do data/teste/)
python scripts/ver_etapas.py          # pontos de cada objeto depois de cada etapa
python scripts/exp_voxel_normais.py   # tabelas e gráficos em data/resultados/

# Testes de regressão (uns 15 s, não precisam de data/)
pytest
```

## Estrutura

```
instor-percepcao/
├── pyproject.toml
├── pyrightconfig.json
├── percepcao/
│   ├── nuvem.py            # classe Nuvem (pontos + rótulos + normais/cores/origens), carregar_gt
│   ├── simulacao/
│   │   ├── malha.py        #   malha do cilindro com 128 faces, criar_cilindro
│   │   ├── cenas.py        #   CILINDROS, VISTAS_TUBOS, GALHO, montar_tubos, montar_galho, salvar_gt
│   │   ├── camera.py       #   capturar, depth_para_metros, intrinsecos
│   │   ├── projecao.py     #   foto -> pontos 3D no mundo; fotografar(vistas) -> Nuvem
│   │   └── trajetoria.py   #   pose_aranha e os parâmetros da aranha
│   ├── etapas/
│   │   ├── filtros.py      #   remover_outliers, reduzir_voxel
│   │   ├── normais.py      #   estimar_normais
│   │   └── primitivas.py   #   contrato do ajuste de primitivas (a implementar)
│   ├── avaliacao.py        # conferir, medir_quadro, normais_verdadeiras, erro_normais, resumo_normais
│   └── pipeline.py         # vai encadear outliers -> voxel -> normais -> primitivas (a implementar)
├── scripts/                # o que se RODA: tudo que tem if __name__ == "__main__"
├── tests/test_regressao.py # os números de referência têm que continuar batendo
└── data/                   # gerado, fora do git
    ├── teste/              #   conjunto de teste fixo (gerar_teste.py)
    ├── resultados/         #   saída dos experimentos
    └── galho/              #   quadros da aranha
```

## Convenções

- **Biblioteca × script.** Os módulos de `percepcao/` só definem funções e
  constantes, sem `if __name__ == "__main__"` e sem abrir janela. Tudo que
  roda ou mostra alguma coisa fica em `scripts/`.
- **O combinado das etapas** (detalhes em `percepcao/nuvem.py`):
  1. toda nuvem é uma `Nuvem`, e os rótulos (o id do objeto de cada ponto)
     viajam junto por todas as etapas;
  2. todo filtro é `etapa(nuvem, parametro=valor) -> Nuvem` e não altera a
     nuvem de entrada;
  3. o ajuste de primitivas devolve uma lista de dicionários **no mesmo formato
     do GT**.
- **Um formato só de GT**, que sai direto de `montar_tubos()` e de `montar_galho()`:

  ```python
  {"unidade": "m", "primitivas": [
      {"tipo": "plano", "nome": "piso", "id_pybullet": 0,
       "normal": [0, 0, 1], "d": 0},            # normal · x + d = 0
      {"tipo": "cilindro", "nome": "pilar", "id_pybullet": 1,
       "raio": ..., "altura": ..., "centro": [x, y, z], "eixo": [ex, ey, ez]},
  ]}
  ```

  Tudo em metros, no referencial do mundo; eixo e normal com tamanho 1.
- **Fotografar e salvar** é sempre `projecao.fotografar(vistas, near, prof_max)`
  seguido de `Nuvem.salvar(caminho)`. O `.ply` guarda pontos, cores e normais;
  rótulos e origens vão em `_rotulos.npy` e `_origens.npy` ao lado.
- Código e comentários em português, explicando o porquê das contas.

## Equipe

- Arthur da Rocha Isoppo
- Carlos Eduardo Lopes Feijó
