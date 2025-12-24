from __future__ import annotations

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path


class PathRecorder:
    def __init__(self, frame_id: str, max_len: int):
        self._frame_id = frame_id
        self._max_len = max_len

        self.path = Path()  # create path object for rviz
        self.path.header.frame_id = frame_id

    def add_xy(self, x: float, y: float, stamp_msg) -> None:
        pose = PoseStamped()
        pose.header.stamp = stamp_msg
        pose.header.frame_id = self._frame_id
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0

        self.path.poses.append(pose)

        # last 100000 poses
        if len(self.path.poses) > self._max_len:
            self.path.poses.pop(0)

        self.path.header.stamp = stamp_msg
