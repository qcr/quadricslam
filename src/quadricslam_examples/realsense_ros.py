import inspect
import os
import textwrap


def run():
    # Print a verbose note each time this is run about ROS workspace & Kimera
    print(
        "%s\n\nThe script can be run via:\n\t%s DESTINATION\n" %
        (textwrap.fill(
            "NOTE: This example requires a ROS workspace running in the "
            "background with the RealSense and Kimera VIO ROS nodes running. "
            "A helper script exists to setup a ROS workspace with these "
            "dependencies.",
            width=80), __file__))


if __name__ == '__main__':
    run()
