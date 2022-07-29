#!/usr/bin/env python

from quadricslam import DataAssociator, DataSource, QuadricSlam
from quadricslam.data_source.realsense import RealSense
from quadricslam.visual_odometry.rgbd_cv2 import RgbdCv2


class IterativeAssociator(DataAssociator):
    pass


if __name__ == '__main__':
    q = QuadricSlam(data_source=RealSense(),
                    visual_odometry=RgbdCv2(),
                    associator=IterativeAssociator())
    q.spin()
