from spatialmath import SE3
from typing import Optional
import numpy as np

from quadricslam import QuadricSlam, VisualOdometry

try:
    import cv2
except:
    raise ImportError("Failed to import 'cv2'. Please install from:"
                      "\n\thttps://pypi.org/project/pyrealsense2/")


class RgbdCv2(VisualOdometry):

    def __init__(self) -> None:
        self.odometry = None
        self.prev_gray = None

    def odom(self, inst: QuadricSlam) -> Optional[SE3]:
        # Initialise the odometry estimator if required
        if self.odometry is None:
            i = np.eye(3)
            c = inst.data_source.calib_rgb()
            i[0, 0] = c[0]
            i[1, 1] = c[1]
            i[0, 2] = c[3]
            i[1, 2] = c[4]
            self.odometry = cv2.rgbd.RgbdOdometry_create(i)

        # Bail early if we're missing images
        if (inst.state_now is None or inst.state_now.rgb is None or
                inst.state_now.depth is None or inst.state_prev is None or
                self.prev_gray is None or inst.state_prev.depth is None):
            return (SE3() if inst.state_now is None or
                    inst.state_now.odom is None else inst.state_now.odom)

        # Compute an odometry estimate using grayscale version of the RGB image
        t = np.eye(4)
        mask = np.ones(inst.state_now.rgb.shape, np.uint8)
        self.odometry.compute(
            self.prev_gray, inst.state_prev.depth, mask,
            cv2.cvtColor(inst.state_now.rgb, cv2.COLOR_RGB2GRAY),
            inst.state_now.depth, mask, t)
        return SE3(t)
