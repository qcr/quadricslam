from typing import Callable, List
import gtsam
import gtsam_quadrics
import numpy as np

from .quadricslam_states import QuadricSlamState

QuadricInitialiser = Callable[
    [List[gtsam.Pose3], List[gtsam_quadrics.AlignedBox2], QuadricSlamState],
    gtsam_quadrics.ConstrainedDualQuadric]


def initialise_quadric_from_depth(
        obs_poses: List[gtsam.Pose3],
        boxes: List[gtsam_quadrics.AlignedBox2],
        state: QuadricSlamState,
        object_depth=0.1) -> gtsam_quadrics.ConstrainedDualQuadric:
    # Uses the depth image to initialise a quadric from a single view (note:
    # this assumes there is only a single view, and will discard all extra
    # views)
    s = state.system
    assert s.calib_rgb is not None
    assert state.this_step is not None
    n = state.this_step
    assert n.depth is not None

    box = boxes[0]
    pose = obs_poses[0]
    calib = gtsam.Cal3_S2(s.calib_rgb)

    # get average box depth
    dbox = box.vector().astype('int')  # get discrete box bounds
    box_depth = n.depth[dbox[1]:dbox[3], dbox[0]:dbox[2]].mean()

    # compute the 3D point corrosponding to the box center
    center = box.center()
    x = (center[0] - calib.px()) * box_depth / calib.fx()
    y = (center[1] - calib.py()) * box_depth / calib.fy()
    relative_point = gtsam.Point3(x, y, box_depth)
    quadric_center = pose.transformFrom(relative_point)

    # compute quadric rotation using .Lookat
    up_vector = pose.transformFrom(gtsam.Point3(0, -1, 0))
    quadric_rotation = gtsam.PinholeCameraCal3_S2.Lookat(
        pose.translation(), quadric_center, up_vector,
        calib).pose().rotation()
    quadric_pose = gtsam.Pose3(quadric_rotation, quadric_center)

    # compute the quadric radii from the box shape
    tx = (box.xmin() - calib.px()) * box_depth / calib.fx()
    ty = (box.ymin() - calib.py()) * box_depth / calib.fy()
    radii = np.array([np.abs(tx - x), np.abs(ty - y), object_depth])

    return gtsam_quadrics.ConstrainedDualQuadric(quadric_pose, radii)


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


def new_factors(current: gtsam.NonlinearFactorGraph,
                previous: gtsam.NonlinearFactorGraph):
    # Figure out the new factors
    fs = (set([current.at(i) for i in range(0, current.size())]) -
          set([previous.at(i) for i in range(0, previous.size())]))

    # Return a NEW graph with the factors
    out = gtsam.NonlinearFactorGraph()
    for f in fs:
        out.add(f)
    return out


def new_values(current: gtsam.Values, previous: gtsam.Values):
    # Figure out new values
    cps, cqs = ps_and_qs_from_values(current)
    pps, pqs = ps_and_qs_from_values(previous)
    vs = {
        **{k: cps[k] for k in list(set(cps.keys()) - set(pps.keys()))},
        **{k: cqs[k] for k in list(set(cqs.keys()) - set(pqs.keys()))}
    }

    # Return NEW values with each of our estimates
    out = gtsam.Values()
    for k, v in vs.items():
        if type(v) == gtsam_quadrics.ConstrainedDualQuadric:
            v.addToValues(out, k)
        else:
            out.insert(k, v)
    return out


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
