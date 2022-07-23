from abc import ABC, abstractmethod
from typing import List, Tuple

from .detector import Detection


class DataAssociator(ABC):

    def __init__(self) -> None:
        pass

    @abstractmethod
    def associate(
        self, new_detections: List[Detection],
        unassociated_detections: List[Detection]
    ) -> Tuple[List[Detection], List[Detection]]:
        # Returns (new_associated_detections, updated_unassociated_detections)
        pass
