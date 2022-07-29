#!/usr/bin/env python3

from subprocess import check_output
from typing import List, Optional, Tuple, Union, cast
import cv2
import numpy as np
import os
import spatialmath as sm
import sys

from detectron2 import config as d2c
from detectron2 import data as d2d
from detectron2 import engine as d2e
from detectron2 import model_zoo as d2mz
from detectron2.utils import visualizer as d2v
import gtsam

from quadricslam import DataSource, Detection, Detector, QuadricSlam, visualise

import pudb


class TumRgbd(DataSource):

    data_list_type = List[List[Union[float, str]]]

    def __init__(self, path: str, rgb_calib: np.ndarray) -> None:
        # Validate path exists
        self.path = path
        if not os.path.isdir(self.path):
            raise ValueError("Path '%s' does not exist." % self.path)

        # Store camera calibration
        self.rgb_calib = rgb_calib

        # Derive synced dataset (aligning on depth as it always has the least
        # data)
        d = self._file_list('depth')
        self.data = {
            **{
                'depth': d
            },
            **{
                t: TumRgbd._synced_list(self._file_list(t), d) for t in [
                    'rgb', 'accelerometer', 'groundtruth'
                ]
            }
        }
        self.data_length = len(self.data['depth'])
        self.restart()

    def _file_list(self, type: str) -> data_list_type:
        fn = os.path.join(self.path, '%s.txt' % type)
        if not os.path.exists(fn):
            raise ValueError("File '%s' does not exist." % fn)
        return [[
            float(x) if i == 0 else x.decode('utf-8')
            for i, x in enumerate(l.split(b' '))
        ]
                for l in check_output("cat %s | grep -v '^#'" %
                                      fn, shell=True).strip().split(b'\n')]

    @staticmethod
    def _synced_list(candidates: data_list_type,
                     reference: data_list_type) -> data_list_type:
        ts_c = np.asarray([c[0] for c in candidates])
        ts_r = np.asarray([r[0] for r in reference])
        return [
            candidates[i] if i == 0 or
            np.abs(np.asarray([candidates[i][0], candidates[i - 1][0]
                              ])).argmin() == 0 else candidates[i - 1]
            for i in np.searchsorted(ts_c, ts_r)
        ]

    def _gt_to_SE3(self, i: int) -> sm.SE3:
        f = np.asarray(self.data['groundtruth'][i][1:], float)
        return sm.SE3.Rt(sm.UnitQuaternion(f[6], f[3:6]).SO3(), f[0:3])

    def calib_rgb(self) -> np.ndarray:
        return self.rgb_calib

    def done(self) -> bool:
        return self.data_i == self.data_length

    def next(
        self, inst: QuadricSlam
    ) -> Tuple[Optional[sm.SE3], np.ndarray, Optional[np.ndarray]]:
        i = self.data_i
        self.data_i += 1
        return (sm.SE3() if i == 0 else self._gt_to_SE3(i) *
                self._gt_to_SE3(i - 1).inv(),
                cv2.imread(
                    os.path.join(self.path,
                                 cast(str, self.data['rgb'][i][1]))), None)

    def restart(self) -> None:
        self.data_i = 0


class FasterRcnn(Detector):

    def __init__(
            self,
            zoo_model: str = 'COCO-Detection/faster_rcnn_R_50_FPN_1x.yaml',
            detection_thresh: float = 0.5) -> None:
        c = d2c.get_cfg()
        c.merge_from_file(d2mz.get_config_file(zoo_model))
        c.MODEL.ROI_HEADS.SCORE_THRESH_TEST = detection_thresh
        c.MODEL.WEIGHTS = d2mz.get_checkpoint_url(zoo_model)
        self.predictor = d2e.DefaultPredictor(c)
        self.classes = d2d.MetadataCatalog.get(
            self.predictor.cfg.DATASETS.TRAIN[0]).thing_classes

    def detect(self, rgb: Optional[np.ndarray],
               pose_key: int) -> List[Detection]:
        inst = self.predictor(rgb)['instances']
        pred_classes = inst.get('pred_classes').detach().cpu().numpy()
        pred_boxes = inst.get('pred_boxes').tensor.detach().cpu().numpy()
        return [
            Detection(label=self.classes[pred_classes[i]],
                      bounds=pred_boxes[i],
                      pose_key=pose_key) for i in range(0, len(pred_classes))
        ]


if __name__ == '__main__':
    # Confirm dataset path is provided
    if len(sys.argv) != 2:
        print("Path to dataset is a required argument.")
        sys.exit(1)
    dataset_path = sys.argv[1]

    # Pull camera calibration parameters from:
    #  https://vision.in.tum.de/data/datasets/rgbd-dataset/file_formats#intrinsic_camera_calibration_of_the_kinect
    camera_calib = np.array(
        [517.3, 516.5, 0, 318.6, 255.3] if 'freiburg1' in
        dataset_path else [520.9, 521.0, 0, 325.1, 249.7] if 'freiburg2' in
        dataset_path else [535.4, 539.2, 0, 320.1, 247.6] if 'freiburg3' in
        dataset_path else [525.0, 525.0, 0, 319.5, 239.5])

    # Run QuadricSLAM
    q = QuadricSlam(
        data_source=TumRgbd(path=dataset_path, rgb_calib=camera_calib),
        detector=FasterRcnn(),
        # TODO needs a viable data association approach
        on_new_estimate=(
            lambda vals, labels, done: visualise(vals, labels, done)))
    q.spin()
