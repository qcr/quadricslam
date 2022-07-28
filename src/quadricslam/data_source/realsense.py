from spatialmath import SE3
from typing import Optional, Tuple
import numpy as np

from . import DataSource

try:
    import pyrealsense2 as rs
except:
    raise ImportError("Failed to import 'pyrealsense2'. Please install from:"
                      "\n\thttps://pypi.org/project/pyrealsense2/")

# TODO this class currently just grabs the RGB and depth frames. For 435i
# devices there are also IMU frames (frame #2 = Accelerometer, frame #3 =
# Gyroscope). Integration could be done on this data to get a noisy odometry
# estimate.


class RealSense(DataSource):

    def __init__(self) -> None:
        # Setup the camera streams
        self.config = rs.config()
        self.config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16,
                                  15)
        self.config.enable_stream(rs.stream.color, 1280, 720, rs.format.rgb8,
                                  15)
        self.pipeline = rs.pipeline()

    def __enter__(self) -> 'RealSense':
        # Store the camera intrinsics

        # Get the depth scale

        return self

    def next(
        self
    ) -> Tuple[Optional[SE3], Optional[np.ndarray], Optional[np.ndarray]]:
        return super().next()
