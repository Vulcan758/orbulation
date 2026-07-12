#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from builtin_interfaces.msg import Time
from cv_bridge import CvBridge

import cv2
import numpy as np
from pyquaternion import Quaternion
from scipy.spatial.transform import Rotation as R

# --- Your Dataset Class ---
class TUMDataset:
    def __init__(self, data_source):
        self.data_source = data_source

        self.depth_frames = np.loadtxt(fname=f"{self.data_source}/depth.txt", dtype=str)
        self.rgb_frames = np.loadtxt(fname=f"{self.data_source}/rgb.txt", dtype=str)
        self.image_scale = 10
        self.matches = np.array(
            self.associate(
                self.rgb_frames[:, 0].astype(np.float64).tolist(),
                self.depth_frames[:, 0].astype(np.float64).tolist(),
            )
        )
        gt_list = np.loadtxt(fname=self.data_source + "/" + "groundtruth.txt", dtype=str)
        self.gt_poses = self.load_poses(gt_list)

    def load_poses(self, gt_list):
        indices = np.abs(
            (
                np.subtract.outer(
                    gt_list[:, 0].astype(np.float64),
                    self.depth_frames[:, 0].astype(np.float64),
                )
            )
        ).argmin(0)

        xyz = gt_list[indices][:, 1:4]
        rotations = np.array(
            [
                Quaternion(x=x, y=y, z=z, w=w).rotation_matrix
                for x, y, z, w in gt_list[indices][:, 4:]
            ]
        )
        num_poses = rotations.shape[0]
        poses = np.eye(4, dtype=np.float64).reshape(1, 4, 4).repeat(num_poses, axis=0)
        poses[:, :3, :3] = rotations
        poses[:, :3, 3] = xyz
        return poses

    def get_timestamps(self):
        return self.matches[:, 0]

    def associate(self, first_list, second_list, offset=0.0, max_difference=0.2):
        matches = []
        for a in first_list:
            temp_matches = [
                (abs(a - (b + offset)), a, b)
                for b in second_list
                if abs(a - (b + offset)) < max_difference
            ]
            if len(temp_matches) != 0:
                diff, first, second = temp_matches[np.argmin(np.array(temp_matches)[:, 0])]
                matches.append((first, second))
        matches.sort()
        return matches

    def __getitem__(self, idx):
        rgb_id, depth_id = self.matches[idx]
        rgb = cv2.imread(f"{self.data_source}/rgb/{rgb_id:.6f}.png", cv2.IMREAD_UNCHANGED)
        depth = cv2.imread(f"{self.data_source}/depth/{depth_id:.6f}.png", cv2.IMREAD_UNCHANGED)
        return rgb, depth, float(rgb_id), float(depth_id)

    def __len__(self):
        return len(self.matches)


# --- ROS 2 Node ---
class TUMPumpNode(Node):
    def __init__(self, data_source, publish_rate=30.0):
        super().__init__('tum_dataset_node')
        self.dataset = TUMDataset(data_source)
        self.bridge = CvBridge()
        
        # ROS 2 Publishers
        self.rgb_pub = self.create_publisher(Image, '/camera/rgb/image_color', 10)
        self.depth_pub = self.create_publisher(Image, '/camera/depth/image_raw', 10)
        self.odom_pub = self.create_publisher(Odometry, '/ground_truth/odom', 10)
        
        # Timer for loop execution
        timer_period = 1.0 / publish_rate
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.current_idx = 0
        
        # Cache depth timestamps to properly align with ground truth poses
        self.depth_timestamps = self.dataset.depth_frames[:, 0].astype(np.float64)

        self.get_logger().info(f"Initialized TUM Dataset Pump from '{data_source}'. Publishing at {publish_rate} Hz.")

    def timer_callback(self):
        if self.current_idx >= len(self.dataset):
            self.get_logger().info("Dataset playback finished.")
            self.timer.cancel()
            rclpy.shutdown()
            return

        # Fetch data
        # Note: I slightly modified your __getitem__ locally above to also return depth_id
        rgb, depth, rgb_ts, depth_ts = self.dataset[self.current_idx]
        
        # Align ground truth pose using depth timestamp
        pose_idx = np.abs(self.depth_timestamps - depth_ts).argmin()
        pose_matrix = self.dataset.gt_poses[pose_idx]

        # Convert float timestamp to ROS Time
        ros_time = Time()
        ros_time.sec = int(rgb_ts)
        ros_time.nanosec = int((rgb_ts - ros_time.sec) * 1e9)

        # 1. Publish RGB
        if rgb is not None:
            # TUM RGB images are standard 8-bit, 3 channels (BGR via OpenCV)
            rgb_msg = self.bridge.cv2_to_imgmsg(rgb, encoding="bgr8")
            rgb_msg.header.stamp = ros_time
            rgb_msg.header.frame_id = "camera_color_optical_frame"
            self.rgb_pub.publish(rgb_msg)

        # 2. Publish Depth
        if depth is not None:
            # TUM Depth images are 16-bit single channel (16UC1)
            depth_msg = self.bridge.cv2_to_imgmsg(depth, encoding="16UC1")
            depth_msg.header.stamp = ros_time
            depth_msg.header.frame_id = "camera_depth_optical_frame"
            self.depth_pub.publish(depth_msg)

        # 3. Publish Odometry
        odom_msg = Odometry()
        odom_msg.header.stamp = ros_time
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "camera_link"

        # Translation
        odom_msg.pose.pose.position.x = pose_matrix[0, 3]
        odom_msg.pose.pose.position.y = pose_matrix[1, 3]
        odom_msg.pose.pose.position.z = pose_matrix[2, 3]

        # Rotation (Convert 3x3 rotation matrix to quaternion)
        rot = R.from_matrix(pose_matrix[:3, :3])
        qx, qy, qz, qw = rot.as_quat()
        odom_msg.pose.pose.orientation.x = qx
        odom_msg.pose.pose.orientation.y = qy
        odom_msg.pose.pose.orientation.z = qz
        odom_msg.pose.pose.orientation.w = qw

        self.odom_pub.publish(odom_msg)

        self.current_idx += 1


def main(args=None):
    rclpy.init(args=args)
    
    data_source = '/home/mahir/orbulating/dataset/tum/rgbd_dataset_freiburg1_xyz' 
    
    try:
        node = TUMPumpNode(data_source=data_source, publish_rate=30.0)
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error starting node: {e}")
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()