from abc import ABC, abstractmethod
from typing import List, Optional
import numpy as np

from ..quadricslam_states import Detection, QuadricSlamState


class Detector(ABC):

    def __init__(self) -> None:
        pass

    @abstractmethod
    def detect(self, state: QuadricSlamState) -> List[Detection]:
        pass
