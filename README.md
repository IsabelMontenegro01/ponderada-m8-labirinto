# Desafio de navegação autônoma — labirinto (culling_games)

Solução das duas partes da ponderada usando o simulador
[culling_games](https://github.com/rmnicola/culling_games) (ROS 2 + pygame).

| Parte | Executável | O que faz |
|---|---|---|
| 1 — navegação com mapa | `ros2 run maze_navigator parte1` | Lê o mapa (`/get_map`), calcula a rota ótima com **A\*** e conduz o robô até o alvo |
| 2 — mapeamento | `ros2 run maze_navigator parte2` | Explora o labirinto **sem o mapa**, só com sensores, constrói o mapa, salva em CSV e comprova que ele reproduz a rota da parte 1 |

## Demonstração

Os vídeos foram gravados no simulador real com o mapa `1.csv`
(`ros2 run cg maze --map 1.csv`).

### Parte 1: navegação com mapa

O robô recebe o mapa completo do simulador e calcula a rota ótima até o alvo
com A\*. Em seguida, percorre essa rota e chega ao alvo com o menor número de
movimentos possível.

<!-- ARRASTE AQUI O parte1.webm (editor do GitHub) -->

🔗 [Assistir ao vídeo da parte 1 no Google Drive](LINK_DRIVE_PARTE1)

### Parte 2: mapeamento do labirinto

O robô começa sem conhecer o mapa e explora o labirinto usando só os sensores,
construindo o mapa durante a exploração. No fim, planeja a rota usando apenas
o mapa que construiu e a executa até o alvo. Isso comprova que o mapa basta
para reproduzir a rota ótima da parte 1, com o mesmo custo.

<!-- ARRASTE AQUI O parte2.webm (editor do GitHub) -->

🔗 [Assistir ao vídeo da parte 2 no Google Drive](LINK_DRIVE_PARTE2)

## Estrutura

```
src/maze_navigator/
├── maze_navigator/
│   ├── grid.py        # grafo do grid: BFS, A*, conversão rota -> comandos, render ASCII (sem ROS)
│   ├── explorer.py    # exploração por fronteira e construção do mapa (sem ROS)
│   ├── ros_client.py  # nó ROS 2: serviços /move_command, /get_map, /reset e tópico de sensores
│   ├── parte1.py      # executável da parte 1
│   └── parte2.py      # executável da parte 2
└── test/test_algoritmos.py  # testes com um simulador falso (rodam sem ROS)
```

Os algoritmos (`grid.py`, `explorer.py`) não dependem do ROS. O robô entra
neles por uma interface mínima, `move(direction) -> bool` e
`sense() -> dict`. Assim o mesmo código roda no simulador real (via
`ros_client.py`) e nos testes.

## Como executar

Pré-requisitos: ROS 2 (Humble/Jazzy) e `pygame` (no Ubuntu: `sudo apt install python3-pygame`; em outros sistemas, `pip install pygame`).

> **Atenção:** o caminho da pasta do workspace não pode ter espaços nem acentos (ex.: `~/Área de trabalho`), senão o `colcon build` do `cg_interfaces` falha. Use, por exemplo, `~/ponderada-m8-labirinto`.

```bash
# 1. clonar este repositório e o simulador dentro de src/
git clone <url-deste-repositorio> ponderada-m8-labirinto
cd ponderada-m8-labirinto
git clone https://github.com/rmnicola/culling_games.git src/culling_games

# 2. compilar
colcon build
source install/setup.bash
```

Terminal 1, o simulador:

```bash
source install/setup.bash
ros2 run cg maze                 # mapa aleatório
ros2 run cg maze --map 1.csv     # ou um mapa específico
```

Terminal 2, a solução:

```bash
source install/setup.bash
ros2 run maze_navigator parte1
ros2 run maze_navigator parte2
```

As duas partes reiniciam o labirinto (`/reset`) antes de começar, então podem
ser rodadas em sequência no mesmo simulador.

Opções úteis:
- `parte1 --delay 0.1`: deixa o movimento mais lento para visualizar (padrão 0.05 s).
- `parte1 --no-reset`: parte da posição atual em vez de reiniciar.
- `parte2 --delay 0.02`: desacelera a exploração (padrão: sem pausa).
- `parte2 --replay-delay 0.1`: velocidade da rota final, depois da exploração (padrão 0.05 s).
- `parte2 --output meu_mapa.csv`: arquivo onde o mapa construído é salvo (padrão `mapa_construido.csv`).

Testes (não precisam de ROS):

```bash
python -m pytest src/maze_navigator/test
```

## Parte 1: navegação com mapa

**Modelagem como grafo.** Cada célula livre (`f`, `r`, `t`) é um vértice. Duas
células livres vizinhas na horizontal ou vertical são ligadas por uma aresta
de custo 1, porque o robô só anda em 4 direções. As paredes (`b`) não
entram no grafo.

**Algoritmo: A\*** com heurística de distância de Manhattan
(`|Δlinha| + |Δcoluna|`).
- A cada passo, expande o nó com menor `f(n) = g(n) + h(n)`, em que `g` é o
  custo já percorrido e `h` é a estimativa até o alvo.
- Num grid 4-conectado de custo unitário, Manhattan nunca superestima a
  distância real. A heurística é **admissível** (e consistente), por isso a
  rota encontrada é **ótima** (a de menor número de movimentos).
- O nó também roda um **BFS** como referência. Com custo unitário, o BFS
  também é ótimo, mas expande mais nós. O log mostra o custo e os nós
  expandidos de cada um. Os testes verificam que os dois custos são iguais.

Depois, a lista de células é convertida em comandos `up/down/left/right` e
enviada ao `/move_command`. Ao final, o nó confere que `robot_pos == target_pos`.

## Parte 2: mapeamento do labirinto

O robô **não usa `/get_map`** para explorar. Ele só tem:
- **sensores** (`/culling_games/robot_sensors`): conteúdo das 8 células vizinhas;
- **movimento** (`/move_command`): diz se o passo deu certo.

A posição vem de **odometria**. O robô começa em `(0, 0)` no seu próprio
referencial e soma o deslocamento de cada movimento bem-sucedido.

**Algoritmo: exploração por fronteira** (*frontier-based exploration*):
1. Lê os sensores e grava as 8 vizinhas no mapa (`dict[(linha, col)] -> 'b'|'f'|'t'`).
2. **Fronteira** = célula livre conhecida que ainda tem algum vizinho
   (4-conectado) desconhecido.
3. Um BFS sobre o mapa já conhecido encontra a fronteira mais próxima. O robô
   vai até ela lendo os sensores a cada passo. Se a fronteira for revelada no
   meio do caminho, ele replaneja.
4. Quando não há mais fronteiras, o mapa está completo. Toda célula alcançável
   e as paredes em volta dela são conhecidas, então nenhum atalho ficou de
   fora e a rota ótima sobre esse mapa é a rota ótima real.

Detalhe importante: no simulador, pisar no alvo encerra o jogo e bloqueia
novos movimentos. Por isso, durante a exploração o alvo é **detectado pelos
sensores, mas tratado como intransponível**.

**Comprovação** (feita automaticamente ao final de `parte2`):
1. O mapa construído é salvo em `mapa_construido.csv`, no mesmo formato dos
   mapas do simulador (`?` = desconhecido).
2. A rota é planejada com A\* **usando só o mapa construído**.
3. O labirinto é reiniciado. Só aqui o `/get_map` é consultado, e apenas para
   comparar: o nó checa que (a) toda célula mapeada bate com o mapa real,
   (b) o alvo está na posição certa e (c) o custo da rota é igual ao da
   parte 1.
4. A rota planejada no mapa construído é executada no simulador e o robô
   chega ao alvo.

## Resultados nos mapas do simulador

Valores obtidos com o simulador falso dos testes, sobre os CSVs de
`culling_games/src/cg/maps`:

| Mapa | Rota ótima (parte 1) | Rota no mapa construído (parte 2) | Passos de exploração |
|---|---|---|---|
| 1.csv | 102 | 102 | 635 |
| 2.csv | 102 | 102 | 705 |
| 3.csv | 192 | 192 | 677 |
| 4.csv | 114 | 114 | 697 |
| 5.csv | 204 | 204 | 619 |
| 6.csv | 110 | 110 | 607 |
| 7.csv | 96 | 96 | 707 |
| 8.csv | 164 | 164 | 653 |
| 9.csv | 190 | 190 | 709 |
| 10.csv | 48 | 48 | 643 |
| test.csv | 25 | 25 | 390 |

Os testes automatizados também cobrem labirintos gerados aleatoriamente, com
e sem ciclos (vários caminhos possíveis até o alvo).
