"""Run ORB-SLAM3 (stereo-inertial) against a TUM-VI rosbag.

The SLAM node needs ~40 s to load the ORB vocabulary before it subscribes;
the bag is started automatically after `bag_delay` seconds so no data is
lost. Set play_bag:=false to manage the bag yourself.

Examples:
  ros2 launch orbulation tumvi_bag.launch.py
  ros2 launch orbulation tumvi_bag.launch.py visualization:=true rate:=0.5
  ros2 launch orbulation tumvi_bag.launch.py \
      bag:=/path/to/other_bag params_file:=/path/to/params.yaml
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    default_params = os.path.join(
        get_package_share_directory("orbulation"), "config", "corridor_stereo_imu.yaml"
    )

    params_file = LaunchConfiguration("params_file")
    bag = LaunchConfiguration("bag")
    rate = LaunchConfiguration("rate")
    bag_delay = LaunchConfiguration("bag_delay")
    visualization = ParameterValue(LaunchConfiguration("visualization"), value_type=bool)

    slam = Node(
        package="orbulation",
        executable="orbslam_ros",
        name="orbslam_ros",
        output="screen",
        parameters=[params_file, {"visualization": visualization}],
    )

    play = TimerAction(
        period=bag_delay,
        actions=[
            ExecuteProcess(
                cmd=["ros2", "bag", "play", bag, "--rate", rate],
                output="screen",
                condition=IfCondition(LaunchConfiguration("play_bag")),
            )
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("params_file", default_value=default_params),
            DeclareLaunchArgument(
                "bag",
                default_value="/home/mahir/orbulating/dataset/tum/dataset-corridor4_512_16",
            ),
            DeclareLaunchArgument("rate", default_value="1.0"),
            DeclareLaunchArgument("play_bag", default_value="true"),
            DeclareLaunchArgument(
                "bag_delay",
                default_value="45.0",
                description="Seconds to wait for the vocabulary load before playing the bag",
            ),
            DeclareLaunchArgument("visualization", default_value="false"),
            slam,
            play,
        ]
    )
