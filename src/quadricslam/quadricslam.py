from spatialmath import SE3
from typing import List, Optional
import gtsam

from .interfaces.data_source import DataSource
from .interfaces.detector import Detector
from .interfaces.visual_odometry import VisualOdometry


def _X(i: int) -> int:
    return int(gtsam.symbol('x', i))


class QuadricSlam:

    def __init__(self, data_source: DataSource, detector: Optional[Detector],
                 visual_odometry: Optional[VisualOdometry]) -> None:
        self.data_source = data_source
        self.detector = detector
        self.visual_odometry = visual_odometry

        self.reset()

    def spin(self) -> None:
        while not self.data_source.done():
            self.step()

    def step(self) -> None:
        # Get latest data from the scene (odom, images, and detections)
        odom, rgb, depth = self.data_source.next()
        if self.visual_odometry:
            odom = self.visual_odometry.odom(rgb, depth, odom)
        detections = self.detector.detect(rgb) if self.detector else []

        # Add new pose to the factor graph
        if self.i == 0:
            self.graph.add(
                gtsam.PriorFactorPose3(_X(self.i), gtsam.Pose3(SE3().A)))
        else:
            self.graph.add(
                gtsam.BetweenFactorPose3(
                    _X(self.i - 1), _X(self.i),
                    gtsam.Pose3((odom if odom else SE3()).A)))

        # Add any newly associated quadric observations to the factor graph
        # TODO

        self.i += 1

    def reset(self) -> None:
        self.data_source.restart()

        self.graph = gtsam.NonlinearFactorGraph()
        self.estimates = gtsam.Values()

        self.i = 0
