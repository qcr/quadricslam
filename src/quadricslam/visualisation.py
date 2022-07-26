import gtsam
import matplotlib.pyplot as plt
import numpy as np

from .quadricslam import QuadricSlam
from .utils import ps_and_qs_from_values

import pudb


def visualise(inst: QuadricSlam, block: bool = False):
    # Get latest pose & quadric estimates
    full_ps, full_qs = ps_and_qs_from_values(inst.estimates)
    ps = [p.matrix() for p in full_ps.values()]

    pxs, pys, pzs, pxus, pxvs, pxws, pyus, pyvs, pyws, pzus, pzvs, pzws = (
        [p[0, 3] for p in ps],
        [p[1, 3] for p in ps],
        [p[2, 3] for p in ps],
        [p[0, 0] for p in ps],
        [p[1, 0] for p in ps],
        [p[2, 0] for p in ps],
        [p[0, 1] for p in ps],
        [p[1, 1] for p in ps],
        [p[2, 1] for p in ps],
        [p[0, 2] for p in ps],
        [p[1, 2] for p in ps],
        [p[2, 2] for p in ps],
    )

    plt.gca(projection='3d')
    # plt.plot(pxs, pys, pzs, color='k')
    # plt.quiver(pxs, pys, pzs, pxus, pxvs, pxws, color='r')
    # plt.quiver(pxs, pys, pzs, pyus, pyvs, pyws, color='g')
    # plt.quiver(pxs, pys, pzs, pzus, pzvs, pzws, color='b')

    for q in full_qs.values():
        visualise_ellipsoid(q.pose().matrix(), q.radii(), 'k')

    plt.show(block=block)

    # pu.db


def visualise_ellipsoid(pose: np.ndarray, radii: np.ndarray, color):
    # Generate ellipsoid of appropriate size at origin
    u, v = np.linspace(0, 2 * np.pi, 100), np.linspace(0, np.pi, 100)
    x, y, z = (radii[0] * np.outer(np.cos(u), np.sin(v)),
               radii[1] * np.outer(np.sin(u), np.sin(v)),
               radii[2] * np.outer(np.ones_like(u), np.cos(v)))

    # Rotate the ellipsoid, then translate to centroid
    # TODO

    # Plot the ellipsoid
    plt.gca().plot_surface(
        x,
        y,
        z,
        rstride=4,
        cstride=4,
        edgecolors=color,
        alpha=0,
        linewidth=0.5,
    )
