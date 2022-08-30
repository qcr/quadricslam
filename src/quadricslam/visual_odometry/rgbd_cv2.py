from spatialmath import SE3
from typing import Optional
import numpy as np

from ..quadricslam_states import QuadricSlamState
from . import VisualOdometry

try:
    import cv2
except:
    raise ImportError("Failed to import 'cv2'. Please install from:"
                      "\n\thttps://pypi.org/project/pyrealsense2/")


class RgbdCv2(VisualOdometry):

    def __init__(self) -> None:
        self.odometry = None
        self.prev_odom = None

    def odom(self, state: QuadricSlamState) -> SE3:
        n = state.this_step
        p = state.prev_step
        s = state.system

        # Perform no corrections if we're missing images or RGB calibration
        if (s.calib_rgb is None or n is None or p is None or n.rgb is None or
                p.rgb is None or n.depth is None or p.depth is None):
            return VisualOdometry.safe_odom(
                None, None if n is None else n.odom,
                SE3() if p is None or p.odom is None else p.odom)

        # Initialise the odometry estimator if required
        if self.odometry is None:
            i = np.eye(3)
            i[0, 0] = s.calib_rgb[0]
            i[1, 1] = s.calib_rgb[1]
            i[0, 2] = s.calib_rgb[3]
            i[1, 2] = s.calib_rgb[4]
            self.odometry = cv2.rgbd.RgbdOdometry_create(i)

        # Compute an odometry estimate using grayscale version of the RGB image
        t = np.eye(4)
        mask = np.ones(n.rgb.shape[:-1], np.uint8)
        self.odometry.compute(cv2.cvtColor(p.rgb,
                                           cv2.COLOR_RGB2GRAY), p.depth, mask,
                              cv2.cvtColor(n.rgb, cv2.COLOR_RGB2GRAY), n.depth,
                              mask, t)
        self.prev_odom = (SE3() if self.prev_odom is None else self.prev_odom *
                          SE3(t))
        return self.prev_odom
