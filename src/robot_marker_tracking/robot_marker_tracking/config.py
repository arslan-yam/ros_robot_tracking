from __future__ import annotations

import numpy as np
from .types import HSVRange

# node / topics / frames
NODE_NAME = "marker_tracker_node"
IMAGE_TOPIC = "/overhead_camera/image"
PATH_TOPIC = "/robot_path"
FRAME_ID = "map"

# publish every 1 sec
PUBLISH_PERIOD_SEC = 1.0

# last 100000 poses
MAX_PATH_LEN = 100000

# find right parameters
MARGIN_OF_ERROR_AREA = 50
HSV_RANGE = HSVRange(
    lower=np.array([0, 0, 0]),
    upper=np.array([180, 255, 70]),
)

# calibration points (pixel -> world)
PIX_PTS = np.array(
    [
        [478.0, 399.0],  # (-2,-2)
        [160.0,  80.0],  # ( 2, 2)
        [160.0, 399.0],  # (-2, 2)
    ],
    dtype=np.float32,
)
WORLD_PTS = np.array(
    [
        [-2.0, -2.0],
        [ 2.0,  2.0],
        [-2.0,  2.0],
    ],
    dtype=np.float32,
)
