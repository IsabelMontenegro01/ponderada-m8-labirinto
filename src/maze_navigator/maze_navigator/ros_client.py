"""Nó ROS 2 que encapsula a comunicação com o simulador culling_games.

Serviços usados:
    /move_command  (cg_interfaces/srv/MoveCmd)
    /get_map       (cg_interfaces/srv/GetMap)
    /reset         (cg_interfaces/srv/Reset)
Tópico assinado:
    /culling_games/robot_sensors  (cg_interfaces/msg/RobotSensors)
"""
from __future__ import annotations

import time
from typing import Dict, List, Tuple

import rclpy
from cg_interfaces.msg import RobotSensors
from cg_interfaces.srv import GetMap, MoveCmd, Reset
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

SENSOR_FIELDS = ('up', 'down', 'left', 'right',
                 'up_left', 'up_right', 'down_left', 'down_right')


class MazeClient(Node):
    def __init__(self, name: str = 'maze_navigator', step_delay: float = 0.0):
        super().__init__(name)
        self.step_delay = step_delay
        self.move_cli = self.create_client(MoveCmd, '/move_command')
        self.map_cli = self.create_client(GetMap, '/get_map')
        self.reset_cli = self.create_client(Reset, '/reset')
        self.create_subscription(RobotSensors, '/culling_games/robot_sensors',
                                 self._on_sensors, qos_profile_sensor_data)
        self._sensors: Dict[str, str] = {}
        self._sensor_count = 0
        self.last_move = None  # última resposta do /move_command

    # ----------------------------------------------------------- utilidades
    def wait_for_simulator(self) -> None:
        for cli in (self.move_cli, self.map_cli, self.reset_cli):
            while not cli.wait_for_service(timeout_sec=1.0):
                self.get_logger().info(
                    f'Aguardando o serviço {cli.srv_name} (o simulador está '
                    'rodando? `ros2 run cg maze`)')

    def _call(self, cli, request, timeout: float = 10.0):
        future = cli.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        if not future.done() or future.result() is None:
            raise RuntimeError(f'Falha ao chamar {cli.srv_name}')
        return future.result()

    def _spin_for(self, seconds: float) -> None:
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.01)

    def _on_sensors(self, msg: RobotSensors) -> None:
        self._sensors = {f: getattr(msg, f) for f in SENSOR_FIELDS}
        self._sensor_count += 1

    # ------------------------------------------------------------- serviços
    def get_map(self) -> List[List[str]]:
        resp = self._call(self.map_cli, GetMap.Request())
        rows, cols = (int(x) for x in resp.occupancy_grid_shape)
        flat = list(resp.occupancy_grid_flattened)
        return [flat[i * cols:(i + 1) * cols] for i in range(rows)]

    def reset(self) -> str:
        """Reinicia o labirinto atual (robô volta ao início)."""
        req = Reset.Request()
        req.is_random = False
        req.map_name = ''
        resp = self._call(self.reset_cli, req)
        # O reset é aplicado no loop do pygame; espera ele acontecer.
        self._spin_for(0.5)
        self.sense()
        return resp.loaded_map_name

    def move_raw(self, direction: str):
        req = MoveCmd.Request()
        req.direction = direction
        resp = self._call(self.move_cli, req)
        self.last_move = resp
        if self.step_delay > 0:
            self._spin_for(self.step_delay)
        return resp

    @staticmethod
    def pos(resp) -> Tuple[int, int]:
        return int(resp.robot_pos[0]), int(resp.robot_pos[1])

    # ------------------------------------------ interface usada pelo Explorer
    def move(self, direction: str) -> bool:
        return bool(self.move_raw(direction).success)

    def sense(self, fresh_messages: int = 3, timeout: float = 3.0
              ) -> Dict[str, str]:
        """Retorna uma leitura dos sensores posterior ao último comando.

        O simulador publica a 100 Hz; esperamos algumas mensagens novas para
        descartar leituras que já estavam em trânsito antes do movimento.
        """
        target = self._sensor_count + fresh_messages
        end = time.monotonic() + timeout
        while self._sensor_count < target:
            if time.monotonic() > end:
                raise RuntimeError('Sem dados de /culling_games/robot_sensors')
            rclpy.spin_once(self, timeout_sec=0.05)
        return dict(self._sensors)
