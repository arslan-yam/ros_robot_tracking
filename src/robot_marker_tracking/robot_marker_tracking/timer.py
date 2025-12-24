from __future__ import annotations

from typing import Callable
from rclpy.node import Node


class PeriodicTimer:
    def __init__(self, node: Node, period_sec: float, callback: Callable[[], None]):
        # publish every 1 sec
        self._timer = node.create_timer(period_sec, callback)

    def cancel(self) -> None:
        self._timer.cancel()

    def reset(self) -> None:
        self._timer.reset()
