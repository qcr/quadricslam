from abc import ABC, abstractmethod
from typing import List, Optional
import numpy as np


class Detection:

    def __init__(self, label: str, bounds: np.ndarray, pose_key: int) -> None:
        self.label = label
        self.bounds = bounds
        self.pose_key = pose_key


class Detector(ABC):

    def __init__(self) -> None:
        pass

    @abstractmethod
    def detect(self, rgb: Optional[np.ndarray],
               pose_key: int) -> List[Detection]:
        pass
