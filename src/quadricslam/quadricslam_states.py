from spatialmath import SE3
from typing import Dict, List, Optional, Union
import gtsam
import numpy as np


def qi(i: int) -> int:
    return int(gtsam.symbol('q', i))


def xi(i: int) -> int:
    return int(gtsam.symbol('x', i))


class Detection:

    def __init__(self,
                 label: str,
                 bounds: np.ndarray,
                 pose_key: int,
                 quadric_key: Optional[int] = None) -> None:
        self.label = label
        self.bounds = bounds
        self.pose_key = pose_key
        self.quadric_key = quadric_key


class StepState:

    def __init__(self, i: int) -> None:
        self.i = i
        self.pose_key = xi(i)

        self.rgb: Optional[np.ndarray] = None
        self.depth: Optional[np.ndarray] = None
        self.odom: Optional[SE3] = None

        self.detections: List[Detection] = []
        self.new_associated: List[Detection] = []


class SystemState:

    def __init__(
        self,
        initial_pose: SE3,
        noise_prior: np.ndarray,
        noise_odom: np.ndarray,
        noise_boxes: np.ndarray,
        optimiser_batch: bool,
        optimiser_params: Union[gtsam.ISAM2Params,
                                gtsam.LevenbergMarquardtParams,
                                gtsam.GaussNewtonParams],
    ) -> None:
        self.initial_pose = gtsam.Pose3(initial_pose.A)
        self.noise_prior = gtsam.noiseModel.Diagonal.Sigmas(noise_prior)
        self.noise_odom = gtsam.noiseModel.Diagonal.Sigmas(noise_odom)
        self.noise_boxes = gtsam.noiseModel.Diagonal.Sigmas(noise_boxes)

        self.optimiser_batch = optimiser_batch
        self.optimiser_params = optimiser_params
        self.optimiser_type = (
            gtsam.ISAM2 if type(optimiser_params) == gtsam.ISAM2Params else
            gtsam.GaussNewtonOptimizer if type(optimiser_params)
            == gtsam.GaussNewtonParams else gtsam.LevenbergMarquardtOptimizer)

        self.associated: List[Detection] = []
        self.unassociated: List[Detection] = []

        self.labels: Dict[int, str] = {}

        self.graph = gtsam.NonlinearFactorGraph()
        self.estimates = gtsam.Values()

        self.optimiser = None

        self.calib_depth: Optional[float] = None
        self.calib_rgb: Optional[np.ndarray] = None


class QuadricSlamState:

    def __init__(self, system: SystemState) -> None:
        self.system = system

        self.prev_step: Optional[StepState] = None
        self.this_step: Optional[StepState] = None
