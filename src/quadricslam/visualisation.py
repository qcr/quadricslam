import gtsam
import matplotlib.pyplot as plt

from .quadricslam import QuadricSlam
from .utils import ps_and_qs_from_values

import pudb


def visualise(inst: QuadricSlam):
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
    plt.plot(pxs, pys, pzs, color='k')
    plt.quiver(pxs, pys, pzs, pxus, pxvs, pxws, color='r')
    plt.quiver(pxs, pys, pzs, pyus, pyvs, pyws, color='g')
    plt.quiver(pxs, pys, pzs, pzus, pzvs, pzws, color='b')

    pu.db
