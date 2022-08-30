#!/usr/bin/env python

from quadricslam import QuadricSlam, utils

from quadricslam.data_associator.quadric_iou_associator import QuadricIouAssociator
from quadricslam.data_source.realsense import RealSense
from quadricslam.detector.faster_rcnn import FasterRcnn
from quadricslam.visual_odometry.rgbd_cv2 import RgbdCv2
from quadricslam.visualisation import visualise


def run():
    with RealSense() as rs:
        q = QuadricSlam(
            data_source=rs,
            detector=FasterRcnn(),
            visual_odometry=RgbdCv2(),
            associator=QuadricIouAssociator(),
            optimiser_batch=False,
            on_new_estimate=(
                lambda state: visualise(state.system.estimates, state.system.
                                        labels, state.system.optimiser_batch)),
            quadric_initialiser=utils.initialise_quadric_from_depth)
        q.spin()


if __name__ == '__main__':
    run()
