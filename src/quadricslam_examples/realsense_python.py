#!/usr/bin/env python

from quadricslam import DataAssociator, DataSource, QuadricSlam
from quadricslam.data_source.realsense import RealSense


class IterativeAssociator(DataAssociator):
    pass


if __name__ == '__main__':
    q = QuadricSlam(data_source=RealSense(), associator=IterativeAssociator())
    q.spin()
