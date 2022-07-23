from spatialmath import SE3
from typing import List, Optional
import gtsam
import numpy as np

from .interfaces.data_source import DataSource
from .interfaces.detector import Detector
from .interfaces.visual_odometry import VisualOdometry


def qi(i: int) -> int:
    return int(gtsam.symbol('q', i))


def xi(i: int) -> int:
    return int(gtsam.symbol('x', i))


class QuadricSlam:

    def __init__(
        self,
        data_source: DataSource,
        detector: Optional[Detector] = None,
        visual_odometry: Optional[VisualOdometry] = None,
        noise_prior: np.ndarray = np.array([0] * 6, dtype=np.float64),
        noise_odom: np.ndarray = np.array([0.01] * 6, dtype=np.float64),
        noise_boxes: np.ndarray = np.array([3] * 4, dtype=np.float64),
    ) -> None:
        self.data_source = data_source
        self.detector = detector
        self.visual_odometry = visual_odometry

        self.noise_prior = gtsam.noiseModel.Diagonal.Sigmas(noise_prior)
        self.noise_odom = gtsam.noiseModel.Diagonal.Sigmas(noise_odom)
        self.noise_boxes = gtsam.noiseModel.Diagonal.Sigmas(noise_boxes)

        self.reset()

    def guess_initial_values(self) -> None:
        # Guessing approach (only guess values that don't already have an
        # estimate):
        # - guess poses using dead reckoning
        # - guess quadrics using Euclidean mean of all observations
        pass

    def spin(self) -> None:
        while not self.data_source.done():
            self.step()

    def step(self) -> None:
        # Get latest data from the scene (odom, images, and detections)
        odom, rgb, depth = self.data_source.next()
        if self.visual_odometry:
            odom = self.visual_odometry.odom(rgb, depth, odom)
        detections = self.detector.detect(rgb) if self.detector else []

        # # Add new pose to the factor graph
        if self.i == 0:
            self.graph.add(
                gtsam.PriorFactorPose3(xi(self.i), gtsam.Pose3(),
                                       self.noise_prior))
        else:
            self.graph.add(
                gtsam.BetweenFactorPose3(
                    xi(self.i - 1), xi(self.i),
                    gtsam.Pose3((odom if odom else SE3()).A), self.noise_odom))

        # Add any newly associated quadric observations to the factor graph
        # TODO

        self.i += 1

    def reset(self) -> None:
        self.data_source.restart()

        self.graph = gtsam.NonlinearFactorGraph()
        self.estimates = gtsam.Values()

        self.i = 0
