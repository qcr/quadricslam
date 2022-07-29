from abc import ABC, abstractmethod
from spatialmath import SE3
from typing import Optional
import numpy as np

from quadricslam import QuadricSlam


class VisualOdometry(ABC):

    def __init__(self) -> None:
        pass

    @abstractmethod
    def odom(self, inst: QuadricSlam) -> SE3:
        pass
