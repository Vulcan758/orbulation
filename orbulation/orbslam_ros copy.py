import rclpy
from rclpy.node import Node
import numpy as np
import orb_slam3.python_wrapper.orb_slam_pybind as orb
from std_msgs.msg import String
from message_filters import Subscriber, ApproximateTimeSynchronizer
from rclpy.clock import Clock
from sensor_msgs.msg import Image, Imu
import cv_bridge
import threading
from collections import deque
import pooch

# replace with absolute path 
def load_vocabulary():
    extracted_files = pooch.retrieve(
        url="https://github.com/UZ-SLAMLab/ORB_SLAM3/raw/master/Vocabulary/ORBvoc.txt.tar.gz",
        known_hash="sha1:ad82cc2a05da116bf211192e4748024a19b7606e",
        progressbar=True,
        processor=pooch.Untar(members="ORBvoc.txt"),
    )
    return extracted_files[0]


"""
TODO
- for now we're going to go with the default config stuff and use it from the yaml
- do these in the callbacks
--- check how to process each image data as nparray, same with timestamps x
--- convert obtained imu data to nparray x
- check whats with file name
- set proper default topics and also set default paths x
- add helper functions from previous binding used. x
- test all types (mono, monoimu, stereo, stereo imu, etc)
- fix vocab default path thing
- read output of slam thing and properly process into possibly odom topic x --> output of this is SE(3) matrices
- after initial code, we should make it so that the python bindings somehow send the cam info directly
- or we launch can launch a subscriber that listens for camera info and writes to a yaml and also gets 
the path to the yaml and stuff

"""

class ORBSlamROS(Node):
    def __init__(self, config: str,): #config refers to camera info stuff
        super().__init__("orbslam_ros")
        self.bridge = cv_bridge.CvBridge()
        vocabulary = load_vocabulary()

        self.declare_parameter("camera_setup", "stereo")
        self.declare_parameter("imu_used", False)

        self.declare_parameter("visualization", True)

        self.declare_parameter("left_cam_topic", "/camera/left/image_raw")
        self.declare_parameter("right_cam_topic", "/camera/right/image_raw")
        self.declare_parameter("rgb_cam_topic", "/camera/rgb/image_color")
        self.declare_parameter("depth_cam_topic", "/camera/depth/image_raw")
        self.declare_parameter("mono_cam_topic", "/camera/image_raw")
        self.declare_parameter("imu_topic", "/camera/imu")

        # Path variables are best initialized as empty strings if they must be provided externally
        self.declare_parameter("vocabulary_path", vocabulary)

        self.imu_used = self.get_parameter("imu_used").value
        # Track the timestamp of the last processed frame to bound the IMU window
        self.prev_frame_timestamp = None
        
        # Thread-safe buffer for IMU points
        self.imu_lock = threading.Lock()
        self.imu_buffer = deque()

        self.camera_setup = self.get_parameter("camera_setup").value

        queue_size = 10
        max_delay = 0.01

        sensor = self.camera_setup
        if self.imu_used:
            sensor = "imu_" + sensor

        self.supported_sensors = {
            "MONOCULAR": orb._System.eSensor.MONOCULAR,
            "STEREO": orb._System.eSensor.STEREO,
            "RGBD": orb._System.eSensor.RGBD,
            "IMU_MONOCULAR": orb._System.eSensor.IMU_MONOCULAR,
            "IMU_STEREO": orb._System.eSensor.IMU_STEREO,
            "IMU_RGBD": orb._System.eSensor.IMU_RGBD,
        }
        try:
            eSensor = self.supported_sensors[sensor.upper()]
        except KeyError:
            raise ValueError("Invalid camera setup type. Please choose between supported sensors.")

        if self.camera_setup == "stereo":
            self.left_cam_sub = Subscriber(self, Image, self.get_parameter("left_cam_topic").value)
            self.right_cam_sub = Subscriber(self, Image, self.get_parameter("right_cam_topic").value)
            self.stereo_sync = ApproximateTimeSynchronizer([self.left_cam_sub, self.right_cam_sub], 
                                                           queue_size=queue_size, slop=max_delay)
            self.stereo_sync.registerCallback(self.stereo_callback)
            self.get_logger().info("Using Stereo setup")

        if self.camera_setup == "rgbd":
            self.rgb_cam_sub = Subscriber(self, Image, self.get_parameter("rgb_cam_topic").value)
            self.depth_cam_sub = Subscriber(self, Image, self.get_parameter("depth_cam_topic").value)
            self.rgbd_sync = ApproximateTimeSynchronizer([self.rgb_cam_sub, self.depth_cam_sub], 
                                                           queue_size=queue_size, slop=max_delay)
            self.rgbd_sync.registerCallback(self.rgbd_callback)
            self.get_logger().info("Using RGBD setup")

        if self.camera_setup == "monocular":
            self.mono_cam_sub = Subscriber(self, Image, self.get_parameter("mono_cam_topic").value)
            self.get_logger().info("Using Monocular setup")

        if self.imu_used:            
            self.create_subscription(Imu, self.get_parameter("imu_topic").value, self.imu_callback, rclpy.qos.qos_profile_sensor_data)
            self.get_logger().info("IMU used")

        vocabulary = self.get_parameter("vocabulary_path").value
        vis = self.get_parameter("visualization").value
        # config = ""

        self.slam = orb._System(vocabulary, config, eSensor, vis)
        self.get_logger().info("Ready to begin ORB-SLAM ROS HEYYY")


    def monocular_callback(self, msg: Image):
        try:
            t_curr = self.extract_timestamp(msg.header)
            v_imu = self.get_imu_measurements_up_to(t_curr)
            
            cv_img = np.ascontiguousarray(self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8'))
            
            pose = self.slam._TrackMonocular(cv_img, t_curr, v_imu, "")
            self.prev_frame_timestamp = t_curr
        except Exception as e:
            self.get_logger().error(f"Mono failed: {e}")

    def rgbd_callback(self, rgb_msg: Image, depth_msg: Image):
        try:
            t_curr = self.extract_timestamp(rgb_msg.header)
            v_imu = self.get_imu_measurements_up_to(t_curr)
            
            cv_rgb = np.ascontiguousarray(self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding='bgr8'))
            cv_depth = np.ascontiguousarray(self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='16UC1'))
            
            pose = self.slam._TrackRGBD(cv_rgb, cv_depth, t_curr, v_imu, "")
            self.prev_frame_timestamp = t_curr
            print(pose)
            # self.get_logger().info(pose)
        except Exception as e:
            self.get_logger().error(f"RGBD failed: {e}")

    def stereo_callback(self, left_msg: Image, right_msg: Image):
        try:
            t_curr = self.extract_timestamp(left_msg.header)
            v_imu = self.get_imu_measurements_up_to(t_curr)
            
            cv_left = np.ascontiguousarray(self.bridge.imgmsg_to_cv2(left_msg, desired_encoding='bgr8'))
            cv_right = np.ascontiguousarray(self.bridge.imgmsg_to_cv2(right_msg, desired_encoding='bgr8'))
            
            pose = self.slam._TrackStereo(cv_left, cv_right, t_curr, v_imu, "")
            self.prev_frame_timestamp = t_curr
        except Exception as e:
            print("Theres an exeption")
            self.get_logger().error(f"Stereo failed: {e}")
    
    def imu_callback(self, msg: Imu):
        """Quickly captures IMU data packets and saves them to the shared buffer."""
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        
        # Format list to map to C++: ax, ay, az, wx, wy, wz, timestamp
        imu_point = np.array([
            msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z,
            msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z,
            t
        ])
        print(imu_point)
        
        with self.imu_lock:
            self.imu_buffer.append(imu_point)

    def get_imu_measurements_up_to(self, current_timestamp: float) -> np.ndarray:
        """Slices out the relevant time window of IMU inputs for the current frame."""
        if not self.imu_used:
            return np.empty((0, 7), dtype=np.float64)
            
        captured_points = []
        with self.imu_lock:
            # Extract measurements that fall within the tracking timeframe
            while self.imu_buffer and self.imu_buffer[0][6] <= current_timestamp:
                captured_points.append(self.imu_buffer.popleft())
                
        if len(captured_points) == 0:
            return np.empty((0, 7), dtype=np.float64)
            
        return np.array(captured_points, dtype=np.float64)

    def extract_timestamp(self, header) -> float:
        return header.stamp.sec + header.stamp.nanosec * 1e-9
    
    def destroy_node(self):
        self.slam._Shutdown()
        super().destroy_node()

    def ActivateLocalizationMode(self):
        self.slam._ActivateLocalizationMode()

    def DeactivateLocalizationMode(self):
        self.slam._DeactivateLocalizationMode()

    def GetImageScale(self):
        return self.slam._GetImageScale()

    def MapChanged(self):
        return self.slam._MapChanged()

    def Reset(self):
        self.slam._Reset()

    def ResetActiveMap(self):
        self.slam._ResetActiveMap()

    def SaveKeyFrameTrajectoryEuroC(self, filename):
        self.slam._SaveKeyFrameTrajectoryEuroC(filename)

    def SaveTrajectoryEuroC(self, filename):
        self.slam._SaveTrajectoryEuroC(filename)

    def SaveKeyFrameTrajectoryTUM(self, filename):
        self.slam._SaveKeyFrameTrajectoryTUM(filename)

    def SaveTrajectoryKITTI(self, filename):
        self.slam._SaveTrajectoryKITTI(filename)

    def SaveTrajectoryTUM(self, filename):
        self.slam._SaveTrajectoryTUM(filename)

    def Shutdown(self):
        self.slam._Shutdown()

    def isFinished(self):
        return self.slam._isFinished()

    def isLost(self):
        return self.slam._isLost()

    def isShutDown(self):
        return self.slam._isShutDown()

def main(args=None):
    rclpy.init(args=args)
    # node = ORBSlamROS(config="/home/mahir/orbulating/ORB-SLAM-Python/examples/config/TUM1.yaml",)
    # node = ORBSlamROS(config="/home/mahir/orbulating/ORB-SLAM-Python/examples/config/TUM1.yaml",)
    # node = ORBSlamROS(config="/home/mahir/orbulating/ORB-SLAM-Python/examples/config/TUM-VI.yasml",)
    # node = ORBSlamROS(config="/home/mahir/orbulating/orb_ws/src/orbulation/config/camera_params/zed2i.yaml",)
    # node = ORBSlamROS(config="/home/mahir/orbulating/orb_ws/src/orbulation/config/camera_params/zed2i.yaml",)
    node = ORBSlamROS(config="/home/mahir/orbulating/ORB-SLAM-Python/examples/config/TUM-VI.yaml",)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

    if __name__ == '__main__':
        main()