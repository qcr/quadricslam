from abc import ABC, abstractmethod
from typing import List
import numpy as np


class Detector(ABC):

    def __init__(self) -> None:
        pass

    @abstractmethod
    def detect(self, rgb: np.ndarray) -> List[np.ndarray]:
        pass
