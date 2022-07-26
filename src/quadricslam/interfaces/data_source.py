from abc import ABC, abstractmethod
from typing import Optional, Tuple
import numpy as np
import spatialmath as sm

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
    ) -> Tuple[Optional[sm.SE3], Optional[np.ndarray], Optional[np.ndarray]]:
        pass

    @abstractmethod
    def restart(self) -> None:
        pass
