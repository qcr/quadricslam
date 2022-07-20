from typing import Optional

from .interfaces.data_source import DataSource
from .interfaces.visual_odometry import VisualOdometry


class QuadricSlam:

    def __init__(self, data_source: DataSource,
                 visual_odometry: Optional[VisualOdometry]) -> None:
        self.data_source = data_source
        self.visual_odometry = visual_odometry

    def run(self) -> None:
        while not self.data_source.done():
            odom, rgb, depth = self.data_source.next()
            if not odom and self.visual_odometry:
                odom = self.visual_odometry.odom(rgb, depth)
