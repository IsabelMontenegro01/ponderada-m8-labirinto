"""Testes sem ROS: um simulador falso reproduz as regras do culling_games
(sensores de 8 vizinhos, movimento bloqueado por paredes, jogo encerrado ao
pisar no alvo) para validar a parte 1 e a parte 2.

Rodar:  python -m pytest src/maze_navigator/test
"""
import os
import random
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from maze_navigator.explorer import Explorer  # noqa: E402
from maze_navigator.grid import (DIRECTIONS, SENSOR_OFFSETS, a_star, bfs,  # noqa: E402
                                 find, grid_to_dict, path_to_directions)


def generate_maze(size=29, seed=0, extra_openings=0):
    """Mesmo algoritmo (DFS recursivo) do gerador do simulador, com opção de
    abrir paredes extras para criar ciclos (vários caminhos possíveis)."""
    rng = random.Random(seed)
    maze = [['b'] * size for _ in range(size)]
    maze[1][1] = 'f'
    stack = [(1, 1)]
    while stack:
        x, y = stack[-1]
        options = [(x + dx, y + dy) for dx, dy in ((0, 2), (0, -2), (2, 0), (-2, 0))
                   if 0 < x + dx < size - 1 and 0 < y + dy < size - 1
                   and maze[y + dy][x + dx] == 'b']
        if options:
            nx, ny = rng.choice(options)
            maze[ny][nx] = 'f'
            maze[(y + ny) // 2][(x + nx) // 2] = 'f'
            stack.append((nx, ny))
        else:
            stack.pop()
    for _ in range(extra_openings):
        i, j = rng.randrange(1, size - 1), rng.randrange(1, size - 1)
        maze[i][j] = 'f'
    maze[1][1] = 'r'
    maze[size // 2][size // 2] = 't'
    return maze


class FakeRobot:
    def __init__(self, grid):
        self.grid = [row[:] for row in grid]
        self.pos = find(self.grid, 'r')
        self.won = False

    def sense(self):
        return {name: self.grid[self.pos[0] + d[0]][self.pos[1] + d[1]]
                for name, d in SENSOR_OFFSETS.items()}

    def move(self, direction):
        if self.won:
            return False
        d = DIRECTIONS[direction]
        nxt = (self.pos[0] + d[0], self.pos[1] + d[1])
        if self.grid[nxt[0]][nxt[1]] == 'b':
            return False
        if self.grid[nxt[0]][nxt[1]] == 't':
            self.won = True
        self.grid[self.pos[0]][self.pos[1]] = 'f'
        self.grid[nxt[0]][nxt[1]] = 'r'
        self.pos = nxt
        return True


def optimal(grid):
    cells = grid_to_dict(grid)
    return a_star(cells, find(grid, 'r'), find(grid, 't'))


@pytest.mark.parametrize('seed', range(10))
@pytest.mark.parametrize('openings', [0, 40])
def test_parte1_astar_otimo_e_executavel(seed, openings):
    grid = generate_maze(seed=seed, extra_openings=openings)
    cells = grid_to_dict(grid)
    start, goal = find(grid, 'r'), find(grid, 't')
    res_a, res_b = a_star(cells, start, goal), bfs(cells, start, goal)
    assert res_a.cost == res_b.cost          # A* é ótimo (BFS é referência)
    assert res_a.expanded <= res_b.expanded  # e expande no máximo o mesmo

    robot = FakeRobot(grid)
    for move in path_to_directions(res_a.path):
        assert robot.move(move)
    assert robot.pos == goal and robot.won


@pytest.mark.parametrize('seed', range(10))
@pytest.mark.parametrize('openings', [0, 40])
def test_parte2_mapa_construido_reproduz_rota(seed, openings):
    grid = generate_maze(seed=seed, extra_openings=openings)
    real_start = find(grid, 'r')
    robot = FakeRobot(grid)
    explorer = Explorer(robot)
    explorer.explore()

    assert not robot.won  # nunca pisou no alvo durante a exploração
    assert explorer.target is not None

    # Toda célula mapeada confere com o mapa real.
    for (i, j), v in explorer.map.items():
        real = grid[i + real_start[0]][j + real_start[1]]
        assert (v == 'f' and real in 'fr') or v == real

    # A rota planejada no mapa construído tem o mesmo custo da parte 1...
    plan = explorer.plan_to_target()
    assert plan.cost == optimal(grid).cost

    # ...e, executada num labirinto reiniciado, chega ao alvo.
    fresh = FakeRobot(grid)
    for move in path_to_directions(plan.path):
        assert fresh.move(move)
    assert fresh.won


def test_csv_do_mapa_construido():
    grid = generate_maze(seed=3)
    explorer = Explorer(FakeRobot(grid))
    explorer.explore()
    built = explorer.as_grid()
    oi, oj = explorer.origin_offset()
    assert built[oi][oj] == 'r'
    # O mapa construído, lido como se fosse um CSV do simulador, tem a mesma
    # rota ótima que o original.
    assert optimal(built).cost == optimal(grid).cost
