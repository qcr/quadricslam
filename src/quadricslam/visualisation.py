from distinctipy import get_colors
from matplotlib.patches import Patch
from typing import Dict
import gtsam
import matplotlib.pyplot as plt
import numpy as np

from .utils import ps_and_qs_from_values

import pudb


def _axis_limits(ps, qs):
    xs = ([p.translation()[0] for p in ps] + [q.bounds().xmin() for q in qs] +
          [q.bounds().xmax() for q in qs])
    ys = ([p.translation()[1] for p in ps] + [q.bounds().ymin() for q in qs] +
          [q.bounds().ymax() for q in qs])
    return np.min(xs), np.max(xs), np.min(ys), np.max(ys)


def _scale_factor(ps, qs):
    lims = _axis_limits(ps, qs)
    return np.max([lims[1] - lims[0], lims[3] - lims[2]])


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


def visualise(values: gtsam.Values,
              labels: Dict[int, str],
              block: bool = False):
    # Generate colour swatch for our labels
    ls = set(labels.values())
    cs = {l: c for l, c in zip(ls, get_colors(len(ls)))}

    # Get latest pose & quadric estimates
    full_ps, full_qs = ps_and_qs_from_values(values)
    sf = 0.1 * _scale_factor(full_ps.values(), full_qs.values())
    ps = [p.matrix() for p in full_ps.values()]

    pxs, pys, pzs, pxus, pxvs, pxws, pyus, pyvs, pyws, pzus, pzvs, pzws = (
        np.array([p[0, 3] for p in ps]),
        np.array([p[1, 3] for p in ps]),
        np.array([p[2, 3] for p in ps]),
        np.array([p[0, 0] for p in ps]),
        np.array([p[1, 0] for p in ps]),
        np.array([p[2, 0] for p in ps]),
        np.array([p[0, 1] for p in ps]),
        np.array([p[1, 1] for p in ps]),
        np.array([p[2, 1] for p in ps]),
        np.array([p[0, 2] for p in ps]),
        np.array([p[1, 2] for p in ps]),
        np.array([p[2, 2] for p in ps]),
    )

    ax = plt.gca(projection='3d')
    ax.clear()
    alphas = np.linspace(0.2, 1, len(ps))
    for i in range(1, len(ps)):
        plt.plot(pxs[i - 1:i + 1],
                 pys[i - 1:i + 1],
                 pzs[i - 1:i + 1],
                 color='k',
                 alpha=alphas[i])
    plt.quiver(pxs, pys, pzs, pxus * sf, pxvs * sf, pxws * sf, color='r')
    plt.quiver(pxs, pys, pzs, pyus * sf, pyvs * sf, pyws * sf, color='g')
    plt.quiver(pxs, pys, pzs, pzus * sf, pzvs * sf, pzws * sf, color='b')

    for k, q in full_qs.items():
        visualise_ellipsoid(q.pose().matrix(), q.radii(), cs[labels[k]])

    # Plot a legend for quadric colours
    ax.legend(handles=[
        Patch(facecolor=c, edgecolor=c, label=l) for l, c in cs.items()
    ])

    # Show the final thing, blocking if requested
    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_zlabel('z')
    ax.autoscale()
    _set_axes_equal(ax)
    plt.show(block=block)
    plt.pause(0.05)


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
