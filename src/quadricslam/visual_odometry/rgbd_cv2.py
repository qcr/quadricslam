from spatialmath import SE3
from typing import Optional
import numpy as np

from . import VisualOdometry

try:
    import cv2
except:
    raise ImportError("Failed to import 'cv2'. Please install from:"
                      "\n\thttps://pypi.org/project/pyrealsense2/")


class RgbdCv2(VisualOdometry):

    def __init__(self) -> None:
        self.odometry = None
        self.calib = None
        self.prev_rgb = None
        self.prev_depth = None

    def odom(self, rgb: np.ndarray, depth: Optional[np.ndarray],
             odom_raw: Optional[SE3], rgb_calib: np.ndarray) -> Optional[SE3]:
        # Initialise the odometry estimator if required
        if self.odometry is None or self.calib is None:
            i = np.eye(3)
            i[0, 0] = rgb_calib[0]
            i[1, 1] = rgb_calib[1]
            i[0, 2] = rgb_calib[3]
            i[1, 2] = rgb_calib[4]
            self.odometry = cv2.rgbd.RgbdOdometry_create(i)

        # Bail early if we're missing images
        if (rgb is None or depth is None or self.prev_rgb is None or
                self.prev_depth):
            self.prev_rgb = rgb
            self.prev_depth = depth
            return None

        # Compute an odometry estimate using grayscale version of the RGB image
        t = np.eye(4)
        mask = np.ones(rgb.shape, np.uint8)
        self.odometry.compute(self.prev_rgb, self.prev_depth, mask,
                              cv2.cvtColor(rgb, cv2.COLOR_RGB2GRAY), depth,
                              mask, t)

        # Cache our images and return the result
        self.prev_rgb = rgb
        self.prev_depth = depth
        return SE3(t)
