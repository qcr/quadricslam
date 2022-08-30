#!/usr/bin/env python3

import gtsam
import gtsam_quadrics
import numpy as np

from quadricslam import visualise


def run():
    # Noise models & shortcuts for generating symbols
    NOISE_PRIOR = gtsam.noiseModel.Diagonal.Sigmas(
        np.array([1e-1] * 6, dtype=np.float64))
    NOISE_ODOM = gtsam.noiseModel.Diagonal.Sigmas(
        np.array([0.01] * 6, dtype=np.float64))
    NOISE_BBOX = gtsam.noiseModel.Diagonal.Sigmas(
        np.array([3] * 4, dtype=np.float64))

    X = lambda i: int(gtsam.symbol('x', i))
    Q = lambda i: int(gtsam.symbol('q', i))

    # Define the ground truth for our dummy scene:
    # - a trajectory of poses tracing a diamond around the origin
    # - each pose faces towards the origin
    # - 2 quadrics around the origin
    poses = [
        gtsam.PinholeCameraCal3_S2.Lookat(x, [0, 0, 0], [0, 0, 1],
                                          gtsam.Cal3_S2()).pose() for x in [
                                              [10, 0, 0],
                                              [0, -10, 0],
                                              [-10, 0, 0],
                                              [0, 10, 0],
                                              [10, 0, 0],
                                          ]
    ]
    quadrics = [
        gtsam_quadrics.ConstrainedDualQuadric(gtsam.Pose3(), [1, 1, 1]),
        gtsam_quadrics.ConstrainedDualQuadric(
            gtsam.Pose3(gtsam.Rot3(), [1, 1, 1]), [1, 1, 1])
    ]

    # Define our graph with:
    # - a prior factor for starting postion
    # - odometry factors between each pose
    # - a bounding box factor for observing the Quadric at each pose
    graph = gtsam.NonlinearFactorGraph()

    graph.add(gtsam.PriorFactorPose3(X(0), poses[0], NOISE_PRIOR))
    for i in range(len(poses) - 1):
        graph.add(
            gtsam.BetweenFactorPose3(X(i), X(i + 1),
                                     poses[i].between(poses[i + 1]),
                                     NOISE_ODOM))

    cal = gtsam.Cal3_S2(525, 525, 0, 160, 120)
    for iq, q in enumerate(quadrics):
        for ip, p in enumerate(poses):
            graph.add(
                gtsam_quadrics.BoundingBoxFactor(
                    gtsam_quadrics.QuadricCamera.project(q, p, cal).bounds(),
                    cal, X(ip), Q(iq), NOISE_BBOX))

    # Start with some rubbish estimates for each of the poses, and use the
    # optimiser to recover the trajectory
    values = gtsam.Values()
    for i, p in enumerate(poses):
        values.insert(X(i), gtsam.Pose3())
    for i, q in enumerate(quadrics):
        q.addToValues(values, Q(i))

    r = gtsam.LevenbergMarquardtOptimizer(graph, values).optimize()

    print(r)
    visualise(r, {Q(i): gtsam.Symbol(Q(i)).string() for i in [0, 1]}, True)


if __name__ == '__main__':
    run()
