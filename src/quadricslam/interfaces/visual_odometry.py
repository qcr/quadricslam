import numpy as np
from abc import ABC, abstractmethod
from typing import Optional


class VisualOdometry(ABC):

    def __init__(self) -> None:
        pass

    @abstractmethod
    def odom(self, rgb: np.ndarray, depth: Optional[np.ndarray]):
        pass
