import numpy as np
from abc import ABC, abstractmethod
from typing import Optional, Tuple, Union

import gtsam


class DataSource(ABC):

    def __init__(self) -> None:
        pass

    @abstractmethod
    def done(self) -> bool:
        pass

    @abstractmethod
    def next(
        self
    ) -> Tuple[Optional[gtsam.Pose3], np.ndarray, Optional[np.ndarray]]:
        pass
