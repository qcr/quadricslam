# QuadricSLAM

[![QUT Centre for Robotics Open Source](https://github.com/qcr/qcr.github.io/raw/master/misc/badge.svg)](https://qcr.github.io)
![Primary language](https://img.shields.io/github/languages/top/qcr/quadricslam)
[![License](https://img.shields.io/github/license/qcr/quadricslam)](./LICENSE.txt)

<!-- [![Conda Version](https://img.shields.io/conda/vn/conda-forge/quadricslam.svg)](https://anaconda.org/conda-forge/quadricslam) -->
<!-- [![Conda Recipe](https://img.shields.io/badge/recipe-quadricslam-green.svg)](https://anaconda.org/conda-forge/quadricslam) -->
<!-- [![Conda Platforms](https://img.shields.io/conda/pn/conda-forge/quadricslam.svg)](https://anaconda.org/conda-forge/quadricslam) -->

This repository contains demonstrations of QuadricSLAM in a variety of different systems. The demonstrations apply our custom [Quadrics extension for GTSAM](https://github.com/qcr/gtsam-quadrics) to simultaneous localisation and mapping (SLAM) problems in a variety of different domains and software ecosystems.

TODO feature image

Systems include:

- a simple "hello_world" example with dummy data
- running QuadricSLAM on the [TUM RGB-D dataset](https://vision.in.tum.de/data/datasets/rgbd-dataset), as done in [our paper](#citing-our-work) TODO
- plug-n-play QuadricSLAM on a [Intel RealSense D435i](https://www.intelrealsense.com/depth-camera-d435i/) TODO
- QuadricSLAM on a [Intel RealSense D435i](https://www.intelrealsense.com/depth-camera-d435i/) in a ROS ecosystem TODO
- QuadricSLAM in photorealistic 3D simulation as an [add-on](https://github.com/qcr/benchbot_addons) for the [BenchBot ecosystem](https://github.com/qcr/benchbot) TODO

## Using the examples in this repository

These examples all assume that you have our [GTSAM Quadrics](https://github.com/qcr/gtsam-quadrics) installed. It is [available on PyPI](https://pypi.org/project/gtsam-quadrics/), and can be installed via:

```
pip install gtsam_quadrics
```

Other installation methods, like from source and Conda are documented in the [GTSAM Quadrics README](https://github.com/qcr/gtsam-quadrics#installation).

## Details of the available examples

TODO when the examples are actually done

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
