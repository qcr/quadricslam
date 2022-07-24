from spatialmath import SE3
from typing import List, Optional
import gtsam
import gtsam_quadrics
import numpy as np

from .interfaces.data_associator import DataAssociator
from .interfaces.data_source import DataSource
from .interfaces.detector import Detection, Detector
from .interfaces.visual_odometry import VisualOdometry


def qi(i: int) -> int:
    return int(gtsam.symbol('q', i))


def xi(i: int) -> int:
    return int(gtsam.symbol('x', i))


class QuadricSlam:

    def __init__(
        self,
        data_source: DataSource,
        visual_odometry: Optional[VisualOdometry] = None,
        detector: Optional[Detector] = None,
        associator: Optional[DataAssociator] = None,
        noise_prior: np.ndarray = np.array([0] * 6, dtype=np.float64),
        noise_odom: np.ndarray = np.array([0.01] * 6, dtype=np.float64),
        noise_boxes: np.ndarray = np.array([3] * 4, dtype=np.float64),
    ) -> None:
        self.data_source = data_source
        self.visual_odometry = visual_odometry
        self.detector = detector
        self.associator = associator

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
        pose_key = xi(self.i)

        # Get latest data from the scene (odom, images, and detections)
        odom, rgb, depth = self.data_source.next()
        if self.visual_odometry:
            odom = self.visual_odometry.odom(rgb, depth, odom)
        detections = self.detector.detect(rgb,
                                          pose_key) if self.detector else []
        new_associated, self.associated, self.unassociated = (
            self.associator.associate(detections, self.associated,
                                      self.unassociated) if self.associator
            else (detections, self.associated + detections, self.unassociated))

        # # Add new pose to the factor graph
        if self.i == 0:
            self.graph.add(
                gtsam.PriorFactorPose3(pose_key, gtsam.Pose3(),
                                       self.noise_prior))
        else:
            self.graph.add(
                gtsam.BetweenFactorPose3(
                    xi(self.i - 1), pose_key,
                    gtsam.Pose3((odom if odom else SE3()).A), self.noise_odom))

        # Add any newly associated detections to the factor graph
        for d in new_associated:
            self.graph.add(
                gtsam_quadrics.BoundingBoxFactor(
                    gtsam_quadrics.AlignedBox2(d.bounds),
                    gtsam.Cal3_S2(self.detector.calib()), d.pose_key,
                    d.quadric_key, self.noise_boxes))

        self.i += 1

    def reset(self) -> None:
        self.data_source.restart()

        self.associated: List[Detection] = []
        self.unassociated: List[Detection] = []

        self.graph = gtsam.NonlinearFactorGraph()
        self.estimates = gtsam.Values()

        self.i = 0
