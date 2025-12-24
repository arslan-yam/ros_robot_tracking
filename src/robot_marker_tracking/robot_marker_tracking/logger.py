from __future__ import annotations

from rclpy.node import Node


class AppLogger:
    def __init__(self, node: Node, component: str = "NODE"):
        self._logger = node.get_logger()
        self._component = component

    def info(self, msg: str) -> None:
        self._logger.info(f"[{self._component}] {msg}")

    def warn(self, msg: str) -> None:
        self._logger.warn(f"[{self._component}] {msg}")

    def error(self, msg: str) -> None:
        self._logger.error(f"[{self._component}] {msg}")

    def debug(self, msg: str) -> None:
        self._logger.debug(f"[{self._component}] {msg}")
