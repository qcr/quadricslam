#!/usr/bin/env python3

from quadricslam import (DataAssociator, DataSource, Detection, Detector,
                         QuadricSlam, qi, visualise)
from spatialmath import SE3
from typing import Any, List, Optional, Tuple
import gtsam
import gtsam_quadrics
import numpy as np

POSES = [
    gtsam.PinholeCameraCal3_S2.Lookat(x, [0, 0, 0], [0, 0, 1],
                                      gtsam.Cal3_S2()).pose() for x in [
                                          [10, 0, 0],
                                          [0, -10, 0],
                                          [-10, 0, 0],
                                          [0, 10, 0],
                                          [10, 0, 0],
                                      ]
]
QUADRICS = [
    gtsam_quadrics.ConstrainedDualQuadric(gtsam.Pose3(), [1, 1, 1]),
    gtsam_quadrics.ConstrainedDualQuadric(gtsam.Pose3(gtsam.Rot3(), [1, 1, 1]),
                                          [1, 1, 1])
]


class DummyAssociator(DataAssociator):

    @staticmethod
    def _flat(list_of_lists: List[List[Any]]) -> List[Any]:
        return [x for xs in list_of_lists for x in xs]

    def associate(
        self, new_detections: List[Detection], unassociated: List[Detection],
        associated: List[Detection]
    ) -> Tuple[List[Detection], List[Detection], List[Detection]]:
        # Use our labels to associate, accepting all quadrics with >= 3
        # observations
        ds = new_detections + unassociated
        newly_associated = [
            d for d in ds if d.label in set(d.label for d in associated) or
            len([x for x in ds if x.label == d.label]) >= 3
        ]
        for d in newly_associated:
            d.quadric_key = gtsam.symbol(d.label[0], int(d.label[1:]))
        return (newly_associated, [
            d for d in new_detections + unassociated + associated
            if d in newly_associated or d in associated
        ], [
            d for d in new_detections + unassociated + associated
            if d not in newly_associated and d not in associated
        ])


class DummyData(DataSource):

    def __init__(self) -> None:
        self.restart()

    def calib_rgb(self) -> np.ndarray:
        return np.array([525, 525, 0, 160, 120])

    def done(self) -> bool:
        return self.i == len(POSES)

    def next(
        self
    ) -> Tuple[Optional[SE3], Optional[np.ndarray], Optional[np.ndarray]]:
        self.i += 1
        return SE3(POSES[self.i - 1].matrix()), None, None

    def restart(self) -> None:
        self.i = 0


class DummyDetector(Detector):

    def __init__(self) -> None:
        self.i = 0

    def detect(self, rgb: Optional[np.ndarray], rgb_calib: np.ndarray,
               pose_key: int) -> List[Detection]:
        i = self.i
        self.i += 1
        return [
            Detection(label=gtsam.Symbol(qi(iq)).string(),
                      bounds=gtsam_quadrics.QuadricCamera.project(
                          q, POSES[i],
                          gtsam.Cal3_S2(rgb_calib)).bounds().vector(),
                      pose_key=pose_key) for iq, q in enumerate(QUADRICS)
        ]


if __name__ == '__main__':
    q = QuadricSlam(
        data_source=DummyData(),
        detector=DummyDetector(),
        associator=DummyAssociator(),
        initial_pose=POSES[0].matrix(),
        on_new_estimate=(
            lambda vals, labels, done: visualise(vals, labels, done)))
    q.spin()
