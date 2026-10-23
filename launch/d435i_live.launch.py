"""Run the Intel RealSense D435i driver plus ORB-SLAM3 live.

Four modes, selected with mode:=<...>. Each one picks a params file from
config/d435i_<mode>.yaml and configures the realsense driver to publish exactly
the streams that mode consumes:

  stereo_inertial  (default)  infra1+infra2+IMU, emitter OFF  -- best for handheld
  stereo                      infra1+infra2,     emitter OFF
  rgbd_inertial               colour+aligned depth+IMU, emitter ON
  rgbd                        colour+aligned depth,     emitter ON

Two driver settings are non-negotiable and are why plain `ros2 launch
realsense2_camera rs_launch.py` does not work for SLAM:

  * unite_imu_method=2 (LINEAR_INTERPOLATION). The D435i publishes accel and
    gyro on separate topics at different rates; without this there is no
    /camera/camera/imu topic at all and the inertial modes have no input.
  * depth_module.emitter_enabled=0 for the infrared modes. The projector paints
    a dot pattern that is static in the world but moves with the camera in the
    image, which destroys stereo feature matching. RGBD modes keep it on
    because the depth stream needs it.

Examples:
  ros2 launch orbulation d435i_live.launch.py
  ros2 launch orbulation d435i_live.launch.py visualization:=true
  ros2 launch orbulation d435i_live.launch.py mode:=rgbd_inertial
  ros2 launch orbulation d435i_live.launch.py slam:=false      # driver only
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

MODES = ("stereo_inertial", "stereo", "rgbd_inertial", "rgbd")


def _driver_params(mode, imu_rate, initial_reset):
    """Stream configuration for the realsense node, per SLAM mode."""
    infrared = mode.startswith("stereo")
    inertial = mode.endswith("inertial")

    params = {
        "enable_sync": True,
        "initial_reset": initial_reset,
        # infrared modes want the projector off; rgbd needs it for depth quality
        "depth_module.emitter_enabled": 0 if infrared else 1,
        "enable_infra1": infrared,
        "enable_infra2": infrared,
        "enable_color": not infrared,
        # the depth stream must stay on in rgbd modes to feed align_depth
        "enable_depth": not infrared,
        "align_depth.enable": not infrared,
        "enable_gyro": inertial,
        "enable_accel": inertial,
        # 0=NONE 1=COPY 2=LINEAR_INTERPOLATION; 2 is what produces /camera/camera/imu
        "unite_imu_method": 2 if inertial else 0,
    }
    if infrared:
        params["depth_module.infra_profile"] = "640x480x30"
    else:
        params["rgb_camera.color_profile"] = "640x480x30"
        params["depth_module.depth_profile"] = "640x480x30"
    if inertial:
        params["gyro_fps"] = imu_rate
        params["accel_fps"] = imu_rate
    return params


def _setup(context, *args, **kwargs):
    mode = LaunchConfiguration("mode").perform(context)
    if mode not in MODES:
        raise RuntimeError(f"mode must be one of {MODES}, got {mode!r}")

    imu_rate = int(LaunchConfiguration("imu_rate").perform(context))
    initial_reset = LaunchConfiguration("initial_reset").perform(context).lower() == "true"

    params_file = LaunchConfiguration("params_file").perform(context)
    if not params_file:
        params_file = os.path.join(
            get_package_share_directory("orbulation"), "config", f"d435i_{mode}.yaml"
        )

    visualization = ParameterValue(LaunchConfiguration("visualization"), value_type=bool)

    # namespace+name give the /camera/camera/... topic names the configs expect
    driver = Node(
        package="realsense2_camera",
        executable="realsense2_camera_node",
        namespace="camera",
        name="camera",
        output="screen",
        parameters=[_driver_params(mode, imu_rate, initial_reset)],
    )

    slam = Node(
        package="orbulation",
        executable="orbslam_ros",
        name="orbslam_ros",
        output="screen",
        parameters=[params_file, {"visualization": visualization}],
        condition=IfCondition(LaunchConfiguration("slam")),
    )

    return [driver, slam]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "mode",
                default_value="stereo_inertial",
                description=f"One of {', '.join(MODES)}",
            ),
            DeclareLaunchArgument(
                "params_file",
                default_value="",
                description="Override the config/d435i_<mode>.yaml params file",
            ),
            DeclareLaunchArgument("visualization", default_value="false"),
            DeclareLaunchArgument(
                "slam",
                default_value="true",
                description="Set false to run only the camera driver",
            ),
            DeclareLaunchArgument(
                "imu_rate",
                default_value="200",
                description="Gyro/accel Hz (200 or 400). Must match IMU.Frequency in the ORB-SLAM3 settings file",
            ),
            DeclareLaunchArgument(
                "initial_reset",
                default_value="false",
                description="Hardware-reset the camera on start; use if a previous run left it wedged",
            ),
            OpaqueFunction(function=_setup),
        ]
    )
