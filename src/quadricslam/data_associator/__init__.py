from abc import ABC, abstractmethod
from typing import List, Tuple

from ..quadricslam_states import Detection, QuadricSlamState


class DataAssociator(ABC):

    def __init__(self) -> None:
        pass

    @abstractmethod
    def associate(
        self, state: QuadricSlamState
    ) -> Tuple[List[Detection], List[Detection], List[Detection]]:
        # Returns a tuple of:
        # - list of the newly associated detections
        # - updated list of associated detections
        # - updated list of unassociated detections
        pass
