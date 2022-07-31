from spatialmath import SE3, UnitQuaternion
from subprocess import check_output
from typing import List, Optional, Tuple, Union, cast
import cv2
import numpy as np
import os

from ..quadricslam_states import QuadricSlamState
from . import DataSource


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

    def _gt_to_SE3(self, i: int) -> SE3:
        f = np.asarray(self.data['groundtruth'][i][1:], float)
        return SE3.Rt(UnitQuaternion(f[6], f[3:6]).SO3(), f[0:3])

    def calib_rgb(self) -> np.ndarray:
        return self.rgb_calib

    def done(self) -> bool:
        return self.data_i == self.data_length

    def next(
        self, state: QuadricSlamState
    ) -> Tuple[Optional[SE3], Optional[np.ndarray], Optional[np.ndarray]]:
        i = self.data_i
        self.data_i += 1
        return (SE3() if i == 0 else self._gt_to_SE3(i) *
                self._gt_to_SE3(i - 1).inv(),
                cv2.imread(
                    os.path.join(self.path,
                                 cast(str, self.data['rgb'][i][1]))), None)

    def restart(self) -> None:
        self.data_i = 0
