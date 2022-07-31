#!/usr/bin/env python

from quadricslam import QuadricSlam, utils

from quadricslam.data_associator.quadric_iou_associator import QuadricIouAssociator
from quadricslam.data_source.realsense import RealSense
from quadricslam.visual_odometry.rgbd_cv2 import RgbdCv2

if __name__ == '__main__':
    q = QuadricSlam(data_source=RealSense(),
                    visual_odometry=RgbdCv2(),
                    associator=QuadricIouAssociator(),
                    quadric_initialiser=utils.initialise_quadric_from_depth)
    q.spin()
