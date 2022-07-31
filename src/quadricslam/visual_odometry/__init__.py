from abc import ABC, abstractmethod
from spatialmath import SE3
from typing import Optional
import numpy as np

from ..quadricslam_states import QuadricSlamState


class VisualOdometry(ABC):

    def __init__(self) -> None:
        pass

    @staticmethod
    def safe_odom(corrected: Optional[SE3], uncorrected: Optional[SE3],
                  prev_odom: SE3) -> SE3:
        # Guarantees something is returned (i.e. not None)... "safe" is
        # probably the wrong word
        return (corrected if corrected is not None else
                uncorrected if uncorrected is not None else prev_odom)

    @abstractmethod
    def odom(self, state: QuadricSlamState) -> SE3:
        pass
