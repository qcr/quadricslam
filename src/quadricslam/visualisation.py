import gtsam
import matplotlib.pyplot as plt
import numpy as np

from .utils import ps_and_qs_from_values

import pudb


def _set_axes_equal(ax):
    # Matplotlib is really ordinary for 3D plots... here's a hack taken from
    # here to get 'square' in 3D:
    #   https://stackoverflow.com/a/31364297/1386784
    '''Make axes of 3D plot have equal scale so that spheres appear as spheres,
    cubes as cubes, etc..  This is one possible solution to Matplotlib's
    ax.set_aspect('equal') and ax.axis('equal') not working for 3D.

    Input
      ax: a matplotlib axis, e.g., as output from plt.gca().
    '''

    x_limits = ax.get_xlim3d()
    y_limits = ax.get_ylim3d()
    z_limits = ax.get_zlim3d()

    x_range = abs(x_limits[1] - x_limits[0])
    x_middle = np.mean(x_limits)
    y_range = abs(y_limits[1] - y_limits[0])
    y_middle = np.mean(y_limits)
    z_range = abs(z_limits[1] - z_limits[0])
    z_middle = np.mean(z_limits)

    # The plot bounding box is a sphere in the sense of the infinity
    # norm, hence I call half the max range the plot radius.
    plot_radius = 0.5 * max([x_range, y_range, z_range])

    ax.set_xlim3d([x_middle - plot_radius, x_middle + plot_radius])
    ax.set_ylim3d([y_middle - plot_radius, y_middle + plot_radius])
    ax.set_zlim3d([z_middle - plot_radius, z_middle + plot_radius])


def visualise(values=gtsam.Values, block: bool = False):
    # Get latest pose & quadric estimates
    full_ps, full_qs = ps_and_qs_from_values(values)
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

    ax = plt.gca(projection='3d')
    plt.plot(pxs, pys, pzs, color='k')
    plt.quiver(pxs, pys, pzs, pxus, pxvs, pxws, color='r')
    plt.quiver(pxs, pys, pzs, pyus, pyvs, pyws, color='g')
    plt.quiver(pxs, pys, pzs, pzus, pzvs, pzws, color='b')

    for q in full_qs.values():
        visualise_ellipsoid(q.pose().matrix(), q.radii(), 'k')

    _set_axes_equal(ax)
    plt.show(block=block)


def visualise_ellipsoid(pose: np.ndarray, radii: np.ndarray, color):
    # Generate ellipsoid of appropriate size at origin
    SZ = 50
    u, v = np.linspace(0, 2 * np.pi, SZ), np.linspace(0, np.pi, SZ)
    x, y, z = (radii[0] * np.outer(np.cos(u), np.sin(v)),
               radii[1] * np.outer(np.sin(u), np.sin(v)),
               radii[2] * np.outer(np.ones_like(u), np.cos(v)))

    # Rotate the ellipsoid, then translate to centroid
    ps = pose @ np.vstack([
        x.reshape(-1),
        y.reshape(-1),
        z.reshape(-1),
        np.ones(z.reshape(-1).shape)
    ])

    # Plot the ellipsoid
    plt.gca().plot_wireframe(
        ps[0, :].reshape(SZ, SZ),
        ps[1, :].reshape(SZ, SZ),
        ps[2, :].reshape(SZ, SZ),
        rstride=4,
        cstride=4,
        edgecolors=color,
        linewidth=0.5,
    )
