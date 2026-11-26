"""Parte 1 — navegação com mapa.

1. Pede o mapa completo ao simulador (/get_map).
2. Modela o grid como grafo e calcula a rota ótima até o alvo com A*
   (heurística de Manhattan). Também roda BFS para comparação.
3. Executa a rota enviando um /move_command por passo.

Uso:  ros2 run maze_navigator parte1 [--no-reset] [--delay 0.05]
"""
from __future__ import annotations

import argparse
import sys
from typing import List

import rclpy

from .grid import (ROBOT, TARGET, a_star, bfs, find, grid_to_dict,
                   path_to_directions, render)
from .ros_client import MazeClient


def execute_route(client: MazeClient, moves: List[str]) -> bool:
    """Envia os movimentos e confirma se o robô terminou sobre o alvo."""
    log = client.get_logger()
    resp = None
    for i, direction in enumerate(moves, 1):
        resp = client.move_raw(direction)
        if not resp.success:
            log.error(f'Movimento {i}/{len(moves)} ({direction}) falhou '
                      f'em {MazeClient.pos(resp)}')
            return False
    if resp is None:
        return False
    reached = MazeClient.pos(resp) == (int(resp.target_pos[0]),
                                       int(resp.target_pos[1]))
    return reached


def solve_with_map(client: MazeClient):
    grid = client.get_map()
    cells = grid_to_dict(grid)
    start, goal = find(grid, ROBOT), find(grid, TARGET)
    if start is None or goal is None:
        raise RuntimeError('Mapa sem robô ou alvo (o jogo já foi vencido? '
                           'rode sem --no-reset)')
    result_bfs = bfs(cells, start, goal)
    result = a_star(cells, start, goal)
    if result.path is None:
        raise RuntimeError('Não existe caminho até o alvo')
    return grid, cells, start, goal, result, result_bfs


def main(args=None) -> int:
    parser = argparse.ArgumentParser(description='Parte 1: navegação com mapa')
    parser.add_argument('--no-reset', action='store_true',
                        help='não reinicia o labirinto antes de começar')
    parser.add_argument('--delay', type=float, default=0.05,
                        help='pausa (s) entre passos, para visualização')
    parsed, ros_args = parser.parse_known_args(
        sys.argv[1:] if args is None else args)

    rclpy.init(args=[sys.argv[0]] + ros_args)
    client = MazeClient('parte1_navegacao_com_mapa', step_delay=parsed.delay)
    log = client.get_logger()
    try:
        client.wait_for_simulator()
        if not parsed.no_reset:
            log.info(f'Labirinto reiniciado: {client.reset()}')

        grid, cells, start, goal, result, result_bfs = solve_with_map(client)
        moves = path_to_directions(result.path)

        log.info(f'Mapa recebido: {len(grid)}x{len(grid[0])}, '
                 f'robô em {start}, alvo em {goal}')
        log.info(f'A*  -> custo {result.cost}, {result.expanded} nós expandidos')
        log.info(f'BFS -> custo {result_bfs.cost}, '
                 f'{result_bfs.expanded} nós expandidos')
        print('\nRota ótima (A*):\n' +
              render(cells, result.path, start, goal) + '\n', flush=True)
        log.info(f'Movimentos: {" ".join(moves)}')

        ok = execute_route(client, moves)
        if ok:
            log.info(f'SUCESSO: alvo alcançado em {len(moves)} movimentos.')
        else:
            log.error('O robô NÃO chegou ao alvo.')
        return 0 if ok else 1
    finally:
        client.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
