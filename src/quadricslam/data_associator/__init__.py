from abc import ABC, abstractmethod
from typing import List, Tuple

from quadricslam import Detection, QuadricSlam


class DataAssociator(ABC):

    def __init__(self) -> None:
        pass

    @abstractmethod
    def associate(
        self, inst: QuadricSlam
    ) -> Tuple[List[Detection], List[Detection], List[Detection]]:
        # Returns a tuple of:
        # - list of the newly associated detections
        # - updated list of associated detections
        # - updated list of unassociated detections
        pass
