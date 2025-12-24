from __future__ import annotations

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped


class PathRecorder:
    def __init__(self, frame_id: str, max_len: int):
        self._max_len = max_len
        self.path = Path()  # create path object for rviz
        self.path.header.frame_id = frame_id

    def add_pose(self, pose: PoseStamped) -> None:
        self.path.poses.append(pose)

        # last 100000 poses
        if len(self.path.poses) > self._max_len:
            self.path.poses.pop(0)

    def set_stamp_from_pose(self, pose: PoseStamped) -> None:
        self.path.header.stamp = pose.header.stamp
