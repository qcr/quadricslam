from itertools import groupby
from types import FunctionType
from spatialmath import SE3
from typing import Callable, Dict, List, Optional, Union
import gtsam
import gtsam_quadrics
import numpy as np

from .interfaces.data_associator import DataAssociator
from .interfaces.data_source import DataSource
from .interfaces.detector import Detection, Detector
from .interfaces.visual_odometry import VisualOdometry

import pudb


def guess_quadric(
        obs_poses: List[gtsam.Pose3], boxes: List[gtsam_quadrics.AlignedBox2],
        calib: gtsam.Cal3_S2) -> gtsam_quadrics.ConstrainedDualQuadric:
    # TODO replace with something less dumb...

    # Get each observation point
    ps = np.array([op.translation() for op in obs_poses])

    # Get each observation direction
    # TODO actually use bounding box rather than assuming middle...
    vs = np.array([op.rotation().matrix()[:, 0] for op in obs_poses])

    # Apply this to compute point closet to where all rays converge:
    #   https://stackoverflow.com/a/52089698/1386784
    i_minus_vs = np.eye(3) - (vs[:, :, np.newaxis] @ vs[:, np.newaxis, :])
    quadric_centroid = np.linalg.lstsq(
        i_minus_vs.sum(axis=0),
        (i_minus_vs @ ps[:, :, np.newaxis]).sum(axis=0),
        rcond=None)[0].squeeze()

    # Fudge the rest for now
    # TODO do better...
    return gtsam_quadrics.ConstrainedDualQuadric(
        gtsam.Rot3(), gtsam.Point3(quadric_centroid), [1, 1, 0.1])


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
        initial_pose: Optional[np.ndarray] = None,
        noise_prior: np.ndarray = np.array([0] * 6, dtype=np.float64),
        noise_odom: np.ndarray = np.array([0.01] * 6, dtype=np.float64),
        noise_boxes: np.ndarray = np.array([3] * 4, dtype=np.float64),
        optimiser_batch: Optional[bool] = None,
        optimiser_params: Optional[Union[gtsam.ISAM2Params,
                                         gtsam.LevenbergMarquardtParams,
                                         gtsam.GaussNewtonParams]] = None,
        on_new_estimate: Optional[Callable[
            [gtsam.Values, Dict[int, str], bool], None]] = None
    ) -> None:
        self.data_source = data_source
        self.visual_odometry = visual_odometry
        self.detector = detector

        # TODO this needs a default data associator, we can't do anything
        # meaningful if this is None...
        if associator is None:
            raise NotImplementedError('No default data associator yet exists, '
                                      'so you must provide one.')
        self.associator = associator

        self.initial_pose = (gtsam.Pose3() if initial_pose is None else
                             gtsam.Pose3(initial_pose))
        self.noise_prior = gtsam.noiseModel.Diagonal.Sigmas(noise_prior)
        self.noise_odom = gtsam.noiseModel.Diagonal.Sigmas(noise_odom)
        self.noise_boxes = gtsam.noiseModel.Diagonal.Sigmas(noise_boxes)

        if (optimiser_batch == True and
                type(optimiser_params) == gtsam.ISAM2Params):
            raise ValueError("ERROR: Can't run batch mode with '%s' params." %
                             type(optimiser_params))
        elif (optimiser_batch == False and optimiser_params is not None and
              type(optimiser_params) != gtsam.ISAM2Params):
            raise ValueError(
                "ERROR: Can't run incremental mode with '%s' params." %
                type(optimiser_params))
        if optimiser_params is None:
            optimiser_params = (gtsam.LevenbergMarquardtParams()
                                if optimiser_batch is not False else
                                gtsam.ISAM2Params())
        self.optimiser_batch = type(optimiser_params) != gtsam.ISAM2
        self.optimiser_params = optimiser_params
        self.optimiser_type = (
            gtsam.ISAM2 if type(optimiser_params) == gtsam.ISAM2 else
            gtsam.GaussNewtonOptimizer if type(optimiser_params)
            == gtsam.GaussNewtonParams else gtsam.LevenbergMarquardtOptimizer)

        self.on_new_estimate = on_new_estimate

        self.reset()

    def guess_initial_values(self) -> None:
        # Guessing approach (only guess values that don't already have an
        # estimate):
        # - guess poses using dead reckoning
        # - guess quadrics using Euclidean mean of all observations
        fs = [self.graph.at(i) for i in range(0, self.graph.nrFactors())]

        # Start with prior factors
        for pf in [
                f for f in fs if type(f) == gtsam.PriorFactorPose3 and
                not self.estimates.exists(f.keys()[0])
        ]:
            self.estimates.insert(pf.keys()[0], pf.prior())

        # Add all between factors one-by-one (should never be any remaining,
        # but if they are just dump them at the origin after the main loop)
        bfs = [f for f in fs if type(f) == gtsam.BetweenFactorPose3]
        done = False
        while not done:
            bf = next((f for f in bfs if self.estimates.exists(f.keys()[0]) and
                       not self.estimates.exists(f.keys()[1])), None)
            if bf is None:
                done = True
                continue
            self.estimates.insert(
                bf.keys()[1],
                self.estimates.atPose3(bf.keys()[0]) * bf.measured())
            bfs.remove(bf)
        for bf in bfs:
            self.estimates.insert(bf.keys()[1], gtsam.Pose3())

        # Add all quadric factors
        _ok = lambda x: x.objectKey()
        bbs = sorted([
            f for f in fs if type(f) == gtsam_quadrics.BoundingBoxFactor and
            not self.estimates.exists(f.objectKey())
        ],
                     key=_ok)
        for qbbs in [list(v) for k, v in groupby(bbs, _ok)]:
            guess_quadric(
                [self.estimates.atPose3(bb.poseKey()) for bb in qbbs],
                [bb.measurement for bb in qbbs],
                gtsam.Cal3_S2(self.detector.calib())).addToValues(
                    self.estimates, qbbs[0].objectKey())

    def spin(self) -> None:
        while not self.data_source.done():
            self.step()

        if self.optimiser_batch:
            self.guess_initial_values()
            self.optimiser = self.optimiser_type(self.graph, self.estimates,
                                                 self.optimiser_params)
            self.estimates = self.optimiser.optimize()

        if self.on_new_estimate:
            self.on_new_estimate(self.estimates, self.labels, True)

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
                                      self.unassociated))

        # Extract some labels
        # TODO handle cases where different labels used for a single quadric???
        self.labels = {
            d.quadric_key: d.label
            for d in self.associated
            if d.quadric_key is not None
        }

        # # Add new pose to the factor graph
        if self.i == 0 or self.prev_odom == None:
            self.graph.add(
                gtsam.PriorFactorPose3(pose_key, self.initial_pose,
                                       self.noise_prior))
        else:
            self.graph.add(
                gtsam.BetweenFactorPose3(
                    xi(self.i - 1), pose_key,
                    gtsam.Pose3(
                        (self.prev_odom.inv() * odom if odom else SE3()).A),
                    self.noise_odom))

        # Add any newly associated detections to the factor graph
        for d in new_associated:
            self.graph.add(
                gtsam_quadrics.BoundingBoxFactor(
                    gtsam_quadrics.AlignedBox2(d.bounds),
                    gtsam.Cal3_S2(self.detector.calib()), d.pose_key,
                    d.quadric_key, self.noise_boxes))

        self.i += 1
        self.prev_odom = SE3() if odom is None else odom

    def reset(self) -> None:
        self.data_source.restart()

        self.associated: List[Detection] = []
        self.unassociated: List[Detection] = []

        self.labels: Dict[int, str] = {}

        self.graph = gtsam.NonlinearFactorGraph()
        self.estimates = gtsam.Values()

        self.optimiser = (None if self.optimiser_batch else
                          self.optimiser_type(self.optimiser_params))

        self.i = 0
        self.prev_odom = None
