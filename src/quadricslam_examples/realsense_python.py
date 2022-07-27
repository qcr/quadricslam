#!/usr/bin/env python

from quadricslam import DataAssociator, DataSource, QuadricSlam


class IterativeAssociator(DataAssociator):
    pass


class RealSense(DataSource):
    pass


if __name__ == '__main__':
    q = QuadricSlam(data_source == RealSense(),
                    associator=IterativeAssociator())
    q.spin()
