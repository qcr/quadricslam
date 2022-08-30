from scipy.optimize import linear_sum_assignment
from typing import List, Tuple
import gtsam
import gtsam_quadrics
import numpy as np

from ..quadricslam_states import Detection, QuadricSlamState, qi as QI
from ..utils import ps_and_qs_from_values
from . import DataAssociator

# Basic associator that makes associations by checking the 2D bounding box IOU
# of projections of an existing map of quadrics into the current RGB frame.
# Realistically, will have best chance of working when using an iterative
# optimiser (because the map we're associating against will be best quality)

# NOTE: this method never leaves any detections unassociated between iterations


class QuadricIouAssociator(DataAssociator):

    def __init__(self, iou_thresh: float = 0.2) -> None:
        self.iou_thresh = iou_thresh

    def associate(
        self, state: QuadricSlamState
    ) -> Tuple[List[Detection], List[Detection], List[Detection]]:
        assert state.system.calib_rgb is not None
        assert state.this_step is not None
        assert state.this_step.odom is not None

        s = state.system
        n = state.this_step
        ps, qs = ps_and_qs_from_values(state.system.estimates)
        next_q = (0 if len(qs.values()) == 0 else (
            max([int(gtsam.Symbol(k).string()[1:]) for k in qs.keys()]) + 1))

        # Bail early if there's no quadrics yet to match against (each box is
        # treated as a new quadric)
        if len(qs.values()) == 0:
            for i, d in enumerate(n.detections):
                d.quadric_key = QI(next_q + i)
            return (n.detections, n.detections + s.associated, [])

        # Compute IOU matrix for each unassociated detection
        ious = np.zeros((len(n.detections), len(qs.values())))
        for i, b in enumerate(
            [gtsam_quadrics.AlignedBox2(d.bounds) for d in n.detections]):
            for j, q in enumerate(qs.values()):
                # Note: smartBounds() can be used here for more accuracy?
                ious[i, j] = b.iou(
                    gtsam_quadrics.QuadricCamera.project(
                        q, gtsam.Pose3(n.odom),
                        gtsam.Cal3_S2(s.calib_rgb)).bounds())
                if np.isnan(ious[i, j]):
                    ious[i, j] = 0

        # Solve as an optimal assignment problem
        dis, qis = linear_sum_assignment(-ious)

        # Use IOU thresh to decide whether to associate with an existing
        # quadric, or define a new one
        i = 0
        for di, qi in zip(dis, qis):
            if ious[di, qi] < self.iou_thresh:
                n.detections[di].quadric_key = QI(next_q + i)
                i += 1
            else:
                n.detections[di].quadric_key = list(qs.keys())[qi]

        return (n.detections, n.detections + s.associated, [])
