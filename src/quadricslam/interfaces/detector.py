from abc import ABC, abstractmethod
from typing import List, Optional, TypedDict
import numpy as np


class Detection(TypedDict):
    label: str
    bounds: np.ndarray


class Detector(ABC):

    def __init__(self) -> None:
        pass

    @abstractmethod
    def detect(self, rgb: Optional[np.ndarray]) -> List[Detection]:
        pass
