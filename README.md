<p align=center><strong>~Please note this is only a very early <em>beta</em> release at this stage~</strong></p>

# QuadricSLAM

[![QUT Centre for Robotics Open Source](https://github.com/qcr/qcr.github.io/raw/master/misc/badge.svg)](https://qcr.github.io)
![Primary language](https://img.shields.io/github/languages/top/qcr/quadricslam)
[![PyPI package](https://img.shields.io/pypi/pyversions/quadricslam)](https://pypi.org/project/quadricslam/)
[![License](https://img.shields.io/github/license/qcr/quadricslam)](./LICENSE.txt)

QuadricSLAM is a system for using [quadrics](https://en.wikipedia.org/wiki/Quadric) to represent objects in a scene, leveraging common optimisation tools for simultaneous localisation and mapping (SLAM) problems to converge on stable object maps and camera trajectories. This library uses [Georgia Tech's Smoothing and Mapping (GTSAM)](https://github.com/borglab/gtsam) library for factor graph optimisation, and adds support through our custom [GTSAM quadrics](https://github.com/qcr/gtsam-quadrics) extension.

TODO update with a more holistic reflection of the repository in its current state
[![@youtube QuadricSLAM demonstration for RA-L](https://github.com/best-of-acrv/gtsam-quadrics/raw/master/doc/quadricslam_video.png)](https://www.youtube.com/watch?v=n-j0DFDFSKU)

The key features of this repository are:

- modular abstractions that allow building QuadricSLAM solutions with custom tools:
  ```python
  q = QuadricSLAM(data_source=MyDataSource(), detector=MyDetector(), associator=MyDataAssociator())
  q.spin()
  ```
- basic Matplotlib visualisation routines
- a rich set of plug-n-play examples of the QuadricSLAM system:
  - simple "hello_world" examples with dummy data
  - running on the [TUM RGB-D dataset](https://vision.in.tum.de/data/datasets/rgbd-dataset), as done in [our paper](#citing-our-work) TODO
  - plug-n-play on a [Intel RealSense D435i](https://www.intelrealsense.com/depth-camera-d435i/) with Python TODO
  - plug-n-play on a [Intel RealSense D435i](https://www.intelrealsense.com/depth-camera-d435i/) in a ROS ecosystem TODO
  - using data from photorealistic 3D simulation through an [add-on](https://github.com/qcr/benchbot_addons) for the [BenchBot ecosystem](https://github.com/qcr/benchbot) TODO

We expect this repository to be active and continually improved upon. If you have any feature requests or experience any bugs, don't hesitate to let us know. Our code is free to use, and licensed under BSD-3. We simply ask that you [cite our work](#citing-our-work) if you use QuadricSLAM in your own research.

## Installation and using the library

Pre-build wheels of this library are [available on PyPI](https://pypi.org/project/quadricslam/) for most Linux systems, as well as source distributions. Install the library with:

```
pip install quadricslam
```

From here basic custom QuadricSLAM systems can be setup by implementing and integrating the following abstract classes:

```python
from quadricslam import DataSource, Detector, Associator, visualise

class MyDataSource(DataSource):
  ...

class MyDetector(Detector):
  ...

class MyAssociator(Associator):
  ...

q = QuadricSlam(data_source=MyDataSource(),
                detector=MyDetector(),
                associator=MyAssociator(),
                on_new_estimate=lambda vals, labels, done: visualise(vals, labels, done)))
                )
q.spin()
```

The examples described below also provide code showing how to create customisations for a range of different scenarios.

## Running the examples from this repository

TODO when the examples are actually done

### `hello_manual_quadricslam` and `hello_quadricslam`

TODO

### `tum_rgbd_dataset`

TODO

<p align="center">
<img alt="TUM RGBD QuadricSLAM still 1" src="https://github.com/qcr/quadricslam/wiki/quadricslam_still1.png" width="400"/>
<img alt="TUM RGBD QuadricSLAM still 2" src="https://github.com/qcr/quadricslam/wiki/quadricslam_still2.png"  width="400"/>
</p>

### TODO RealSense and BenchBot examples

## Citing our work

If you are using this library in academic work, please cite the [publication](https://ieeexplore.ieee.org/document/8440105):

L. Nicholson, M. Milford and N. Sünderhauf, "QuadricSLAM: Dual Quadrics From Object Detections as Landmarks in Object-Oriented SLAM," in IEEE Robotics and Automation Letters, vol. 4, no. 1, pp. 1-8, Jan. 2019, doi: 10.1109/LRA.2018.2866205. [PDF](https://arxiv.org/abs/1804.04011).

```bibtex
@article{nicholson2019,
  title={QuadricSLAM: Dual Quadrics From Object Detections as Landmarks in Object-Oriented SLAM},
  author={Nicholson, Lachlan and Milford, Michael and Sünderhauf, Niko},
  journal={IEEE Robotics and Automation Letters},
  year={2019},
}
```
