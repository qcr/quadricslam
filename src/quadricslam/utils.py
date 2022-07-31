from typing import Callable, List
import gtsam
import gtsam_quadrics

from .quadricslam_states import QuadricSlamState

QuadricInitialiser = Callable[
    [List[gtsam.Pose3], List[gtsam_quadrics.AlignedBox2], QuadricSlamState],
    gtsam_quadrics.ConstrainedDualQuadric]


def initialise_quadric_ray_intersection(
        obs_poses: List[gtsam.Pose3], boxes: List[gtsam_quadrics.AlignedBox2],
        state: QuadricSlamState) -> gtsam_quadrics.ConstrainedDualQuadric:
    # Takes all observations of a quadric, projects rays into 3D space, and
    # uses their closest convergence point to place the quadric. Initial
    # orientation and size are currently just dumb guesses.

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


def ps_and_qs_from_values(values: gtsam.Values):
    # TODO there's got to be a better way to access the typed values...
    return ({
        k: values.atPose3(k)
        for k in values.keys()
        if gtsam.Symbol(k).string()[0] == 'x'
    }, {
        k: gtsam_quadrics.ConstrainedDualQuadric.getFromValues(values, k)
        for k in values.keys()
        if gtsam.Symbol(k).string()[0] == 'q'
    })
