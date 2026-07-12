"""Run the SDK-less ZED 2i driver plus ORB-SLAM3 (stereo-inertial) live.

Both nodes read config/zed_stereo_params.yaml (topics, calibrated
image_time_offset, ORB-SLAM3 settings path). The SLAM node takes ~40 s to
load the vocabulary; the driver starts immediately, and the SLAM node
subscribes only once it is ready, so startup order does not matter.

Examples:
  ros2 launch orbulation zed_live.launch.py
  ros2 launch orbulation zed_live.launch.py visualization:=true
  ros2 launch orbulation zed_live.launch.py slam:=false   # driver only
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    default_params = os.path.join(
        get_package_share_directory("orbulation"), "config", "zed_stereo_params.yaml"
    )

    params_file = LaunchConfiguration("params_file")
    visualization = ParameterValue(LaunchConfiguration("visualization"), value_type=bool)

    zed = Node(
        package="orbulation",
        executable="zed_stereo_ros",
        name="zed_stereo_ros",
        output="screen",
        parameters=[params_file],
    )

    slam = Node(
        package="orbulation",
        executable="orbslam_ros",
        name="orbslam_ros",
        output="screen",
        parameters=[params_file, {"visualization": visualization}],
        condition=IfCondition(LaunchConfiguration("slam")),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("params_file", default_value=default_params),
            DeclareLaunchArgument("visualization", default_value="false"),
            DeclareLaunchArgument("slam", default_value="true"),
            zed,
            slam,
        ]
    )
