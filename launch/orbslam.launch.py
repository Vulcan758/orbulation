import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Find the config file dynamically
    config_file = os.path.join(
        get_package_share_directory('orbulation'),
        'config',
        # 'orbslam_params.yaml'
        # 'zed_stereo_params.yaml'
        # 'tum_vi_stereo_imu_params.yaml'
        # 'd435i_rgbd_inertial.yaml'
        'd435i_rgbd.yaml'
    )

    orbslam_node = Node(
        package='orbulation',
        executable='orbslam_ros',
        name='orbslam_ros',
        output='screen',
        parameters=[config_file] # Pass the YAML file directly here
    )

    return LaunchDescription([
        orbslam_node
    ])