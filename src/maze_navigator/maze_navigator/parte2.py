"""Parte 2 — mapeamento do labirinto.

1. Reinicia o labirinto e explora SEM usar /get_map: só sensores + movimento
   (ver explorer.py). O mapa vai sendo construído durante a navegação.
2. Salva o mapa construído em CSV (mesmo formato dos mapas do simulador).
3. Comprovação: planeja a rota com A* usando apenas o mapa construído,
   reinicia o labirinto e executa essa rota até o alvo.
   Só nesta etapa de verificação o /get_map é consultado, para comparar a rota
   do mapa construído com a rota ótima da parte 1 (mesmo custo) e conferir
   que todas as células mapeadas batem com o mapa real.

Uso:  ros2 run maze_navigator parte2 [--output mapa_construido.csv]
                                      [--delay 0.0] [--replay-delay 0.05]
"""
from __future__ import annotations

import argparse
import csv
import sys

import rclpy

from .explorer import Explorer
from .grid import BLOCKED, FREE, ROBOT, path_to_directions, render
from .parte1 import execute_route, solve_with_map
from .ros_client import MazeClient


def main(args=None) -> int:
    parser = argparse.ArgumentParser(description='Parte 2: mapeamento')
    parser.add_argument('--output', default='mapa_construido.csv',
                        help='arquivo CSV onde o mapa construído é salvo')
    parser.add_argument('--delay', type=float, default=0.0,
                        help='pausa (s) entre passos durante a exploração')
    parser.add_argument('--replay-delay', type=float, default=0.05,
                        help='pausa (s) entre passos ao executar a rota final')
    parsed, ros_args = parser.parse_known_args(
        sys.argv[1:] if args is None else args)

    rclpy.init(args=[sys.argv[0]] + ros_args)
    client = MazeClient('parte2_mapeamento', step_delay=parsed.delay)
    log = client.get_logger()
    try:
        client.wait_for_simulator()
        log.info(f'Labirinto reiniciado: {client.reset()}')

        # ---------------------------------------------------- 1. exploração
        explorer = Explorer(client, log=log.info)
        explorer.explore()
        if explorer.target is None:
            log.error('Exploração terminou sem encontrar o alvo.')
            return 1

        # -------------------------------------------- 2. salva o mapa em CSV
        built = explorer.as_grid()
        with open(parsed.output, 'w', newline='') as f:
            csv.writer(f).writerows(built)
        log.info(f'Mapa construído salvo em {parsed.output} '
                 f'({len(built)}x{len(built[0])})')

        # ------------------------- 3. rota usando apenas o mapa construído
        plan = explorer.plan_to_target()
        if plan is None or plan.path is None:
            log.error('O mapa construído não contém caminho até o alvo.')
            return 1
        moves = path_to_directions(plan.path)
        print('\nMapa construído + rota planejada sobre ele:\n' +
              render(explorer.map, plan.path, explorer.start, explorer.target)
              + '\n', flush=True)

        # --------------------------- 4. verificação contra a parte 1
        log.info(f'Reiniciando para reproduzir a rota: {client.reset()}')
        _, real_cells, real_start, real_goal, opt, _ = solve_with_map(client)

        # Converte do referencial do robô (início = (0,0)) para o do mapa real.
        off = real_start
        def to_real(c):
            return c[0] + off[0], c[1] + off[1]

        def norm(v):  # robô e livre são equivalentes para a comparação
            return FREE if v == ROBOT else v

        mismatches = [c for c, v in explorer.map.items()
                      if norm(v) != norm(real_cells.get(to_real(c), BLOCKED))]
        same_target = to_real(explorer.target) == real_goal
        same_path = [to_real(c) for c in plan.path] == opt.path

        log.info(f'Células mapeadas divergentes do mapa real: {len(mismatches)}')
        log.info(f'Alvo mapeado na posição real: {same_target}')
        log.info(f'Custo da rota — parte 1 (mapa real): {opt.cost} | '
                 f'parte 2 (mapa construído): {plan.cost}')
        log.info(f'Rotas idênticas célula a célula: {same_path}')

        # ------------------------------------- 5. executa a rota planejada
        client.step_delay = parsed.replay_delay
        ok = execute_route(client, moves)
        verified = (ok and same_target and not mismatches
                    and plan.cost == opt.cost)
        if verified:
            log.info('COMPROVADO: o mapa construído reproduz a rota ótima da '
                     f'parte 1 ({plan.cost} movimentos) e o robô chegou ao '
                     'alvo.')
        else:
            log.error('Verificação falhou.')
        return 0 if verified else 1
    finally:
        client.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
