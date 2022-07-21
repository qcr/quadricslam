from abc import ABC, abstractmethod
from typing import List, TypedDict
import numpy as np


class Detection(TypedDict):
    class_index: int
    bounds: np.ndarray


class Detector(ABC):

    def __init__(self) -> None:
        pass

    @abstractmethod
    def detect(self, rgb: np.ndarray) -> List[Detection]:
        pass
