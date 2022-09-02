# QuadricSLAM

[![QUT Centre for Robotics Open Source](https://github.com/qcr/qcr.github.io/raw/master/misc/badge.svg)](https://qcr.github.io)
![Primary language](https://img.shields.io/github/languages/top/qcr/quadricslam)
[![PyPI package](https://img.shields.io/pypi/pyversions/quadricslam)](https://pypi.org/project/quadricslam/)
[![License](https://img.shields.io/github/license/qcr/quadricslam)](./LICENSE.txt)

QuadricSLAM is a system for using [quadrics](https://en.wikipedia.org/wiki/Quadric) to represent objects in a scene, leveraging common optimisation tools for simultaneous localisation and mapping (SLAM) problems to converge on stable object maps and camera trajectories. This library uses [Georgia Tech's Smoothing and Mapping (GTSAM)](https://github.com/borglab/gtsam) library for factor graph optimisation, and adds support through our custom [GTSAM quadrics](https://github.com/qcr/gtsam-quadrics) extension.

TODO update with a more holistic reflection of the repository in its current state
[![@youtube QuadricSLAM demonstration for RA-L](https://github.com/qcr/gtsam-quadrics/raw/master/doc/quadricslam_video.png)](https://www.youtube.com/watch?v=n-j0DFDFSKU)

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

_Note: in the spirit of keeping this package light, some dependencies may not be installed; please install those manually_

This repository contains a number of examples to demonstrate how QuadricSLAM systems can be set up in different contexts.

Each example is a file in the `quadricslam_examples` module, with a standalone `run()` function. There are two possible ways to run each example:

1. Directly through the command line:

   ```
   python -m quadricslam_examples.EXAMPLE_NAME ARGS ...
   ```

   e.g for the `hello_quadricslam` examples:

   ```
   python -m quadricslam_examples.hello_quadricslam
   ```

2. Or from within Python:

   ```python
   from quadricslam_examples.EXAMPLE_NAME import run
   run()
   ```

### `hello_manual_quadricslam`

Shows how to create a QuadricSLAM system from scratch using the primitives exposed by our [GTSAM Quadrics library](https://github.com/qcr/gtsam-quadrics). The scenario is 4 viewpoints in a square around 2 quadrics in the middle of the square:

![hello_manual_quadricslam example](https://github.com/qcr/quadricslam/wiki/hello_quadricslam.jpg)

### `hello_quadricslam`

Same scenario as the `hello_manual_quadricslam` example, but uses the abstractions provided by this library. Shows how an entire QuadricSLAM system can be created with only a few lines of code when the appropriate components are available:

![hello_quadricslam example](https://github.com/qcr/quadricslam/wiki/hello_quadricslam.jpg)

### `tum_rgbd_dataset`

Re-creation of the TUM RGBD dataset experiments used in our [initial publication](#citing-our-work). There is a script included for downloading the dataset.

![tum_rgbd_dataset example](https://github.com/qcr/quadricslam/wiki/tum_rgbd.jpg)

_Note: the paper used hand-annotated data to avoid the data association problem; as a result the example here requires a custom data associator to be created before it will run_

### `realsense_python`

Demonstrates how a system can be run using an RGBD RealSense, the [pyrealsense2](https://pypi.org/project/pyrealsense2/) library, and a barebones OpenCV visual odometry algorithm.

The example is a simple plug-n-play system, with weak localisation and data association:

![realsense_python example](https://github.com/qcr/quadricslam/wiki/realsense_python.jpg)

### `realsense_ros`

Demonstrates how a ROS QuadricSLAM system can be put together with an RGBD RealSense, the [ROS RealSense](https://github.com/IntelRealSense/realsense-ros) library, and [Kimera VIO's visual odometry system](https://github.com/MIT-SPARK/Kimera-VIO-ROS).

This example includes a script for creating an entire ROS workspace containing all the required packages built from source. Once installed, it runs the same as the `realsense_python` example but with significantly better localisation:

![realsense_ros example](https://github.com/qcr/quadricslam/wiki/realsense_ros.jpg)

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
