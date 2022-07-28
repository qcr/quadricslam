from abc import ABC, abstractmethod
from spatialmath import SE3
from typing import Optional, Tuple
import numpy as np

import gtsam


class DataSource(ABC):

    def __init__(self) -> None:
        pass

    def calib_depth(self) -> float:
        # Float representing a depth scaling factor
        return 1

    @abstractmethod
    def calib_rgb(self) -> np.ndarray:
        # Vector representing the calibration (fx, fy, skew, u0, v0)
        return np.array([1, 1, 0, 0, 0])

    @abstractmethod
    def done(self) -> bool:
        pass

    @abstractmethod
    def next(
        self
    ) -> Tuple[Optional[SE3], Optional[np.ndarray], Optional[np.ndarray]]:
        pass

    @abstractmethod
    def restart(self) -> None:
        pass
