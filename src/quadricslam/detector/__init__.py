from abc import ABC, abstractmethod
from typing import List, Optional
import numpy as np

from quadricslam import QuadricSlam


class Detection:

    def __init__(self,
                 label: str,
                 bounds: np.ndarray,
                 pose_key: int,
                 quadric_key: Optional[int] = None) -> None:
        self.label = label
        self.bounds = bounds
        self.pose_key = pose_key
        self.quadric_key = quadric_key


class Detector(ABC):

    def __init__(self) -> None:
        pass

    @abstractmethod
    def detect(self, inst: QuadricSlam) -> List[Detection]:
        pass
