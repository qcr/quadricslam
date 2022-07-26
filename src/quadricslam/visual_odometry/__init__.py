from abc import ABC, abstractmethod
from typing import Optional
import numpy as np
import spatialmath as sp


class VisualOdometry(ABC):

    def __init__(self) -> None:
        pass

    @abstractmethod
    def odom(self, rgb: np.ndarray, depth: Optional[np.ndarray],
             odom_raw: Optional[sp.SE3]) -> sp.SE3:
        pass
