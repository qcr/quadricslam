#!/usr/bin/env python3

from quadricslam import (DataAssociator, DataSource, Detection, Detector,
                         QuadricSlam, QuadricSlamState, qi, visualise)
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
        self, state: QuadricSlamState
    ) -> Tuple[List[Detection], List[Detection], List[Detection]]:
        # Use our labels to associate, accepting all quadrics with >= 3
        # observations
        s = state.system
        n = state.this_step
        new_ds = [] if n is None else n.detections

        ds = new_ds + s.unassociated
        newly_associated = [
            d for d in ds if d.label in set(d.label for d in s.associated) or
            len([x for x in ds if x.label == d.label]) >= 3
        ]
        for d in newly_associated:
            d.quadric_key = gtsam.symbol(d.label[0], int(d.label[1:]))
        return (newly_associated, [
            d for d in ds + s.associated
            if d in newly_associated or d in s.associated
        ], [
            d for d in ds + s.associated
            if d not in newly_associated and d not in s.associated
        ])


class DummyData(DataSource):

    def __init__(self) -> None:
        self.restart()

    def calib_rgb(self) -> np.ndarray:
        return np.array([525, 525, 0, 160, 120])

    def done(self) -> bool:
        return self.i == len(POSES)

    def next(
        self, state: QuadricSlamState
    ) -> Tuple[Optional[SE3], Optional[np.ndarray], Optional[np.ndarray]]:
        self.i += 1
        return SE3(POSES[self.i - 1].matrix()), None, None

    def restart(self) -> None:
        self.i = 0


class DummyDetector(Detector):

    def __init__(self) -> None:
        self.i = 0

    def detect(self, state: QuadricSlamState) -> List[Detection]:
        i = self.i
        self.i += 1
        return [
            Detection(label=gtsam.Symbol(qi(iq)).string(),
                      bounds=gtsam_quadrics.QuadricCamera.project(
                          q, POSES[i], gtsam.Cal3_S2(
                              state.system.calib_rgb)).bounds().vector(),
                      pose_key=state.this_step.pose_key)
            for iq, q in enumerate(QUADRICS)
        ]


def run():
    q = QuadricSlam(
        data_source=DummyData(),
        detector=DummyDetector(),
        associator=DummyAssociator(),
        initial_pose=SE3(POSES[0].matrix()),
        optimiser_batch=True,
        on_new_estimate=(
            lambda state: visualise(state.system.estimates, state.system.
                                    labels, state.system.optimiser_batch)))
    q.spin()


if __name__ == '__main__':
    run()
