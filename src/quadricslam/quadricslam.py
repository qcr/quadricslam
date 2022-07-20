from typing import Optional

from .interfaces.datasource import DataSource


class QuadricSlam:

    def __init__(self, datasource: DataSource) -> None:
        self.datasource = datasource

    def run(self) -> None:
        while not self.datasource.done():
            odom, rgb, depth = self.datasource.next()
