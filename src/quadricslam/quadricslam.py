from typing import Optional

from .interfaces.data_source import DataSource
from .interfaces.detector import Detector
from .interfaces.visual_odometry import VisualOdometry


class QuadricSlam:

    def __init__(self, data_source: DataSource, detector: Detector,
                 visual_odometry: Optional[VisualOdometry]) -> None:
        self.data_source = data_source
        self.detector = detector
        self.visual_odometry = visual_odometry

        self.reset()

    def reset(self) -> None:
        self.data_source.restart()

        self.graph = gtsam.NonlinearFactorGraph()
        self.estimates = gtsam.Values()

    def run(self) -> None:
        while not self.data_source.done():
            # Get latest data from the scene (odom, images, and detections)
            odom, rgb, depth = self.data_source.next()
            if self.visual_odometry:
                odom = self.visual_odometry.odom(rgb, depth, odom)
            detections = self.detector.detect(rgb)

            # Add any newly associated quadric observations to the factor graph
            # TODO
