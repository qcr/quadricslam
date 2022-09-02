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
            width=80), os.path.join(os.path.dirname(__file__),
                                    'create_ros_ws')))

    print(
        "%s\n\n\t%s\n\t%s\n\t%s\n" %
        (textwrap.fill(
            "Once this has built there will be a ROS workspace with all of the "
            "manually built dependencies required to run this example. Once the "
            "workspace has been sourced, the following 3 launch files must be run "
            "in order (ensuring there is appropriate delay between each to support "
            "initialisation):",
            width=80), "roslaunch quadricslam_realsense_ros realsense.launch",
         "roslaunch quadricslam_realsense_ros kimera_vio.launch",
         "roslaunch quadricslam_realsense_ros quadricslam.launch"))


if __name__ == '__main__':
    run()
