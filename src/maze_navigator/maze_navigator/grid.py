"""Estruturas e algoritmos de grafo sobre o grid do labirinto.

Este módulo não depende do ROS: o labirinto é tratado como um grafo
implícito em que cada célula livre é um vértice e existe uma aresta (de custo
1) entre duas células livres vizinhas na horizontal ou na vertical.

Convenção de células (a mesma do simulador culling_games):
    'b' -> bloqueada (parede)
    'f' -> livre
    'r' -> robô
    't' -> alvo
    '?' -> desconhecida (usada apenas no mapa construído na parte 2)
"""
from __future__ import annotations

import heapq
from collections import deque
from typing import Callable, Dict, Iterable, List, Optional, Sequence, Tuple

Cell = Tuple[int, int]  # (linha, coluna)

BLOCKED = 'b'
FREE = 'f'
ROBOT = 'r'
TARGET = 't'
UNKNOWN = '?'

# Direção -> deslocamento (d_linha, d_coluna). "up" diminui a linha.
DIRECTIONS: Dict[str, Cell] = {
    'up': (-1, 0),
    'down': (1, 0),
    'left': (0, -1),
    'right': (0, 1),
}

# Nomes dos campos da mensagem RobotSensors -> deslocamento.
SENSOR_OFFSETS: Dict[str, Cell] = {
    'up': (-1, 0),
    'down': (1, 0),
    'left': (0, -1),
    'right': (0, 1),
    'up_left': (-1, -1),
    'up_right': (-1, 1),
    'down_left': (1, -1),
    'down_right': (1, 1),
}


def add(a: Cell, b: Cell) -> Cell:
    return a[0] + b[0], a[1] + b[1]


def manhattan(a: Cell, b: Cell) -> int:
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


def is_passable(value: Optional[str]) -> bool:
    """Uma célula é atravessável se for livre, o robô ou o alvo."""
    return value in (FREE, ROBOT, TARGET)


def find(grid: Sequence[Sequence[str]], value: str) -> Optional[Cell]:
    for i, row in enumerate(grid):
        for j, cell in enumerate(row):
            if cell == value:
                return i, j
    return None


def grid_to_dict(grid: Sequence[Sequence[str]]) -> Dict[Cell, str]:
    return {(i, j): c for i, row in enumerate(grid) for j, c in enumerate(row)}


def neighbors(cells: Dict[Cell, str], cell: Cell,
              passable: Callable[[Optional[str]], bool] = is_passable
              ) -> Iterable[Tuple[str, Cell]]:
    """Vizinhos atravessáveis (4-conectividade) de `cell`."""
    for name, delta in DIRECTIONS.items():
        nxt = add(cell, delta)
        if passable(cells.get(nxt)):
            yield name, nxt


def _rebuild(came_from: Dict[Cell, Optional[Cell]], goal: Cell) -> List[Cell]:
    path = [goal]
    while came_from[path[-1]] is not None:
        path.append(came_from[path[-1]])
    path.reverse()
    return path


class SearchResult:
    """Caminho encontrado (lista de células, incluindo início e fim) e o
    número de nós expandidos pela busca."""

    def __init__(self, path: Optional[List[Cell]], expanded: int):
        self.path = path
        self.expanded = expanded

    @property
    def cost(self) -> Optional[int]:
        return None if self.path is None else len(self.path) - 1


def bfs(cells: Dict[Cell, str], start: Cell, goal: Cell,
        passable: Callable[[Optional[str]], bool] = is_passable
        ) -> SearchResult:
    """Busca em largura. Como todas as arestas têm custo 1, o primeiro
    caminho que chega ao objetivo é ótimo."""
    came_from: Dict[Cell, Optional[Cell]] = {start: None}
    queue = deque([start])
    expanded = 0
    while queue:
        cur = queue.popleft()
        expanded += 1
        if cur == goal:
            return SearchResult(_rebuild(came_from, goal), expanded)
        for _, nxt in neighbors(cells, cur, passable):
            if nxt not in came_from:
                came_from[nxt] = cur
                queue.append(nxt)
    return SearchResult(None, expanded)


def a_star(cells: Dict[Cell, str], start: Cell, goal: Cell,
           passable: Callable[[Optional[str]], bool] = is_passable
           ) -> SearchResult:
    """A* com heurística de Manhattan.

    A distância de Manhattan nunca superestima o custo real num grid
    4-conectado com custo unitário (heurística admissível e consistente),
    então o caminho retornado é ótimo, e em geral expande menos nós que o BFS.
    """
    g: Dict[Cell, int] = {start: 0}
    came_from: Dict[Cell, Optional[Cell]] = {start: None}
    # (f, h, contador, célula) -> desempate por menor h e depois FIFO
    counter = 0
    open_heap = [(manhattan(start, goal), manhattan(start, goal), counter, start)]
    closed = set()
    expanded = 0
    while open_heap:
        _, _, _, cur = heapq.heappop(open_heap)
        if cur in closed:
            continue
        closed.add(cur)
        expanded += 1
        if cur == goal:
            return SearchResult(_rebuild(came_from, goal), expanded)
        for _, nxt in neighbors(cells, cur, passable):
            new_g = g[cur] + 1
            if nxt not in g or new_g < g[nxt]:
                g[nxt] = new_g
                came_from[nxt] = cur
                h = manhattan(nxt, goal)
                counter += 1
                heapq.heappush(open_heap, (new_g + h, h, counter, nxt))
    return SearchResult(None, expanded)


def path_to_directions(path: Sequence[Cell]) -> List[str]:
    """Converte uma sequência de células em comandos up/down/left/right."""
    inverse = {v: k for k, v in DIRECTIONS.items()}
    moves = []
    for a, b in zip(path, path[1:]):
        moves.append(inverse[(b[0] - a[0], b[1] - a[1])])
    return moves


def render(cells: Dict[Cell, str], path: Sequence[Cell] = (),
           start: Optional[Cell] = None, goal: Optional[Cell] = None) -> str:
    """Desenha o grid em ASCII.

    '#' parede, '.' livre, '?' desconhecido, 'S' início, 'T' alvo,
    '*' rota.
    """
    if not cells:
        return ''
    rows = [c[0] for c in cells]
    cols = [c[1] for c in cells]
    on_path = set(path)
    symbols = {BLOCKED: '#', FREE: '.', ROBOT: '.', TARGET: 'T', UNKNOWN: '?'}
    lines = []
    for i in range(min(rows), max(rows) + 1):
        line = []
        for j in range(min(cols), max(cols) + 1):
            cell = (i, j)
            if cell == start:
                ch = 'S'
            elif cell == goal:
                ch = 'T'
            elif cell in on_path:
                ch = '*'
            else:
                ch = symbols.get(cells.get(cell, UNKNOWN), '?')
            line.append(ch)
        lines.append(' '.join(line))
    return '\n'.join(lines)
