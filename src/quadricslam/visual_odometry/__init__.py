from abc import ABC, abstractmethod
from spatialmath import SE3
from typing import Optional
import numpy as np


class VisualOdometry(ABC):

    def __init__(self) -> None:
        pass

    @abstractmethod
    def odom(self, rgb: np.ndarray, depth: Optional[np.ndarray],
             odom_raw: Optional[SE3], rgb_calib: np.ndarray) -> Optional[SE3]:
        pass
