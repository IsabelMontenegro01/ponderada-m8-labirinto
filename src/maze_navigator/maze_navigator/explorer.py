"""Exploração e mapeamento de um labirinto desconhecido (parte 2).

O explorador não sabe nada sobre o labirinto no início. Ele só conta com:
  * os sensores do robô, que informam o conteúdo das 8 células vizinhas;
  * o comando de movimento (up/down/left/right), que diz se o passo deu certo.

A posição é estimada por odometria: o robô começa em (0, 0) no seu próprio
referencial e soma o deslocamento de cada movimento bem-sucedido.

Estratégia: exploração por fronteira (frontier-based exploration).
  * Fronteira = célula livre já conhecida que ainda tem algum vizinho
    (4-conectado) desconhecido.
  * A cada iteração o robô vai, por BFS sobre o mapa conhecido, até a
    fronteira mais próxima, lendo os sensores a cada passo.
  * Quando não há mais fronteiras, todas as células alcançáveis (e as paredes
    em volta delas) são conhecidas, ou seja, o mapa está completo.

O alvo é detectado pelos sensores, mas o robô NUNCA pisa nele durante a
exploração: no simulador, chegar ao alvo encerra o jogo e bloqueia novos
movimentos.
"""
from __future__ import annotations

from collections import deque
from typing import Callable, Dict, List, Optional, Protocol

from .grid import (BLOCKED, DIRECTIONS, FREE, ROBOT, SENSOR_OFFSETS, TARGET,
                   UNKNOWN, Cell, a_star, add, path_to_directions)


class RobotInterface(Protocol):
    """O mínimo que o explorador precisa do robô (ROS ou simulado)."""

    def move(self, direction: str) -> bool:
        ...

    def sense(self) -> Dict[str, str]:
        ...


class Explorer:
    def __init__(self, robot: RobotInterface,
                 log: Callable[[str], None] = lambda _msg: None,
                 max_steps: int = 100_000):
        self.robot = robot
        self.log = log
        self.max_steps = max_steps

        self.start: Cell = (0, 0)
        self.pos: Cell = (0, 0)
        self.map: Dict[Cell, str] = {self.start: FREE}
        self.visited = {self.start}
        self.target: Optional[Cell] = None
        self.steps = 0
        self.trajectory: List[Cell] = [self.start]

    # ------------------------------------------------------------------ mapa
    def _walkable(self, value: Optional[str]) -> bool:
        # Durante a exploração o alvo é tratado como intransponível.
        return value in (FREE, ROBOT)

    def update_from_sensors(self) -> None:
        readings = self.robot.sense()
        for name, delta in SENSOR_OFFSETS.items():
            value = readings.get(name)
            if not value:
                continue
            cell = add(self.pos, delta)
            if value == ROBOT:  # não deveria acontecer, mas por segurança
                value = FREE
            self.map[cell] = value
            if value == TARGET and self.target is None:
                self.target = cell
                self.log(f'Alvo detectado em {cell} (referencial do robô)')
        self.map[self.pos] = FREE

    def is_frontier(self, cell: Cell) -> bool:
        if not self._walkable(self.map.get(cell)):
            return False
        return any(add(cell, d) not in self.map for d in DIRECTIONS.values())

    def frontiers(self) -> List[Cell]:
        return [c for c in self.map if self.is_frontier(c)]

    # ------------------------------------------------------------- movimento
    def _step(self, direction: str) -> bool:
        if self.steps >= self.max_steps:
            raise RuntimeError('Limite de passos atingido na exploração')
        ok = self.robot.move(direction)
        self.steps += 1
        nxt = add(self.pos, DIRECTIONS[direction])
        if ok:
            self.pos = nxt
            self.visited.add(nxt)
            self.trajectory.append(nxt)
        else:
            # Sensor e realidade divergiram: registra parede e replaneja.
            self.map[nxt] = BLOCKED
        self.update_from_sensors()
        return ok

    def _nearest_frontier_path(self) -> Optional[List[Cell]]:
        """BFS a partir da posição atual até a fronteira mais próxima."""
        came_from: Dict[Cell, Optional[Cell]] = {self.pos: None}
        queue = deque([self.pos])
        while queue:
            cur = queue.popleft()
            if self.is_frontier(cur):
                path = [cur]
                while came_from[path[-1]] is not None:
                    path.append(came_from[path[-1]])
                return path[::-1]
            for d in DIRECTIONS.values():
                nxt = add(cur, d)
                if nxt not in came_from and self._walkable(self.map.get(nxt)):
                    came_from[nxt] = cur
                    queue.append(nxt)
        return None

    def explore(self) -> Dict[Cell, str]:
        """Explora até não restarem fronteiras. Retorna o mapa construído."""
        self.update_from_sensors()
        while True:
            path = self._nearest_frontier_path()
            if path is None:
                break
            goal = path[-1]
            for direction in path_to_directions(path):
                if not self._step(direction):
                    break  # replaneja
                # Se a fronteira escolhida já foi revelada no caminho,
                # não precisa terminar o trajeto até ela.
                if self.pos != goal and not self.is_frontier(goal):
                    break
        self.log(f'Exploração concluída: {self.steps} passos, '
                 f'{len(self.visited)} células visitadas, '
                 f'{len(self.map)} células mapeadas')
        return self.map

    # ------------------------------------------------------------ resultado
    def plan_to_target(self, search=a_star):
        """Planeja a rota ótima de start até o alvo usando SÓ o mapa
        construído (desconhecido = intransponível)."""
        if self.target is None:
            return None
        return search(self.map, self.start, self.target)

    def as_grid(self) -> List[List[str]]:
        """Mapa construído como matriz (mesmo formato dos CSVs do
        simulador), recortado ao retângulo conhecido. '?' = desconhecido."""
        rows = [c[0] for c in self.map]
        cols = [c[1] for c in self.map]
        grid = []
        for i in range(min(rows), max(rows) + 1):
            row = []
            for j in range(min(cols), max(cols) + 1):
                cell = (i, j)
                if cell == self.start:
                    row.append(ROBOT)
                else:
                    row.append(self.map.get(cell, UNKNOWN))
            grid.append(row)
        return grid

    def origin_offset(self) -> Cell:
        """Posição de `start` dentro da matriz retornada por as_grid()."""
        return (-min(c[0] for c in self.map), -min(c[1] for c in self.map))
