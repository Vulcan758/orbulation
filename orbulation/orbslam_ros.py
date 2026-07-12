#!/usr/bin/env python3
"""ORB-SLAM3 ROS2 node built on the pyorbslam3 bindings.

Supports all six sensor modes (mono / stereo / rgbd, each with or without
IMU). Every topic is a parameter with a default; see the declare_parameter
block. Publishes:

  odom_topic   nav_msgs/Odometry     pose of the camera in the SLAM world
  pose_topic   geometry_msgs/PoseStamped   same pose, lighter message
  state_topic  std_msgs/Int32        pyorbslam3.TrackingState value

No TF, no map publishing (deliberately out of scope for now).

IMU handling (the part that made the previous node fail): measurements are
buffered on their own executor thread, and each frame consumes exactly the
window (t_prev_frame, t_frame]. The pyorbslam3 bindings release the GIL
during tracking, so the IMU callback genuinely keeps running while a frame
is being tracked — with the old bindings the GIL was held and the buffer
starved. Frames are briefly delayed (never dropped mid-stream) until their
IMU window has arrived.
"""

from __future__ import annotations

import math
import threading
import time
from collections import deque

import numpy as np
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)

import cv_bridge
from geometry_msgs.msg import PoseStamped
from message_filters import ApproximateTimeSynchronizer, Subscriber
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, Imu
from std_msgs.msg import Int32

import pyorbslam3 as pos

DEFAULT_VOCAB = "/home/mahir/orbulating/pyorbslam3/vocabulary/ORBvoc.txt"
DEFAULT_CONFIG = "/home/mahir/orbulating/pyorbslam3/configs/tumvi/stereo_inertial.yaml"

_SENSORS = {
    ("monocular", False): pos.Sensor.MONOCULAR,
    ("monocular", True): pos.Sensor.IMU_MONOCULAR,
    ("stereo", False): pos.Sensor.STEREO,
    ("stereo", True): pos.Sensor.IMU_STEREO,
    ("rgbd", False): pos.Sensor.RGBD,
    ("rgbd", True): pos.Sensor.IMU_RGBD,
}


def _rotation_to_quaternion(R: np.ndarray):
    """3x3 rotation matrix -> (x, y, z, w) quaternion (Shepperd's method)."""
    t = np.trace(R)
    if t > 0.0:
        s = math.sqrt(t + 1.0) * 2.0
        w = 0.25 * s
        x = (R[2, 1] - R[1, 2]) / s
        y = (R[0, 2] - R[2, 0]) / s
        z = (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    return x, y, z, w


class OrbSlamNode(Node):
    def __init__(self):
        super().__init__("orbslam_ros")
        self.bridge = cv_bridge.CvBridge()

        # -- parameters ------------------------------------------------------
        self.declare_parameter("camera_setup", "stereo")  # monocular|stereo|rgbd
        self.declare_parameter("imu_used", True)
        self.declare_parameter("vocabulary_path", DEFAULT_VOCAB)
        self.declare_parameter("config", DEFAULT_CONFIG)
        self.declare_parameter("visualization", False)

        # input topics
        self.declare_parameter("left_cam_topic", "/cam0/image_raw")
        self.declare_parameter("right_cam_topic", "/cam1/image_raw")
        self.declare_parameter("mono_cam_topic", "/cam0/image_raw")
        self.declare_parameter("rgb_cam_topic", "/camera/rgb/image_raw")
        self.declare_parameter("depth_cam_topic", "/camera/depth/image_raw")
        self.declare_parameter("imu_topic", "/imu0")

        # output topics + frames
        self.declare_parameter("odom_topic", "/orbslam/odometry")
        self.declare_parameter("pose_topic", "/orbslam/pose")
        self.declare_parameter("state_topic", "/orbslam/tracking_state")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "camera")

        # behavior knobs
        self.declare_parameter("sync_queue_size", 10)
        self.declare_parameter("sync_slop", 0.01)  # s, stereo/rgbd pair matching
        self.declare_parameter("imu_wait_timeout", 0.15)  # s, wait for IMU coverage
        self.declare_parameter("status_period", 5.0)  # s, periodic status log
        # EuRoC-format trajectory dump on shutdown ('' disables)
        self.declare_parameter("save_trajectory_path", "")

        p = lambda name: self.get_parameter(name).value  # noqa: E731

        self.camera_setup = p("camera_setup").lower()
        self.imu_used = bool(p("imu_used"))
        try:
            sensor = _SENSORS[(self.camera_setup, self.imu_used)]
        except KeyError:
            raise ValueError(
                f"camera_setup must be monocular|stereo|rgbd, got '{self.camera_setup}'"
            )

        self.odom_frame = p("odom_frame")
        self.base_frame = p("base_frame")
        self.imu_wait_timeout = float(p("imu_wait_timeout"))
        self.save_trajectory_path = p("save_trajectory_path")

        # -- publishers ------------------------------------------------------
        self.odom_pub = self.create_publisher(Odometry, p("odom_topic"), 10)
        self.pose_pub = self.create_publisher(PoseStamped, p("pose_topic"), 10)
        self.state_pub = self.create_publisher(Int32, p("state_topic"), 10)

        # -- state -----------------------------------------------------------
        self.imu_lock = threading.Lock()
        self.imu_buffer: deque = deque(maxlen=100000)
        self.prev_frame_timestamp: float | None = None
        self.imu_init_logged = False
        self.frames = 0
        self.frames_ok = 0
        self.frames_skipped = 0
        self.last_state = None
        self._status_wall = time.monotonic()
        self._status_frames = 0

        # -- SLAM system -----------------------------------------------------
        # Construct BEFORE subscribing: the vocabulary load blocks for tens of
        # seconds, and any messages queued during it would be delivered stale,
        # de-synchronizing the IMU from the frames.
        self.get_logger().info(
            f"Starting ORB-SLAM3: sensor={sensor.name}, config={p('config')}"
        )
        self.get_logger().info("Loading vocabulary (takes a while)...")
        self.slam = pos.Slam(
            vocab_file=p("vocabulary_path"),
            settings_file=p("config"),
            sensor=sensor,
            use_viewer=bool(p("visualization")),
        )
        self.get_logger().info("SLAM system ready")

        # -- subscriptions ----------------------------------------------------
        # Tracking is serialized in one mutually-exclusive group; the IMU gets
        # a reentrant group so it keeps buffering while tracking blocks
        # (requires the MultiThreadedExecutor set up in main()).
        self.image_cb_group = MutuallyExclusiveCallbackGroup()
        self.imu_cb_group = ReentrantCallbackGroup()

        queue_size = int(p("sync_queue_size"))
        slop = float(p("sync_slop"))

        if self.camera_setup == "stereo":
            subs = [
                Subscriber(self, Image, p("left_cam_topic"),
                           qos_profile=qos_profile_sensor_data,
                           callback_group=self.image_cb_group),
                Subscriber(self, Image, p("right_cam_topic"),
                           qos_profile=qos_profile_sensor_data,
                           callback_group=self.image_cb_group),
            ]
            self._sync = ApproximateTimeSynchronizer(subs, queue_size, slop)
            self._sync.registerCallback(self.stereo_callback)
        elif self.camera_setup == "rgbd":
            subs = [
                Subscriber(self, Image, p("rgb_cam_topic"),
                           qos_profile=qos_profile_sensor_data,
                           callback_group=self.image_cb_group),
                Subscriber(self, Image, p("depth_cam_topic"),
                           qos_profile=qos_profile_sensor_data,
                           callback_group=self.image_cb_group),
            ]
            self._sync = ApproximateTimeSynchronizer(subs, queue_size, slop)
            self._sync.registerCallback(self.rgbd_callback)
        else:  # monocular
            self.create_subscription(
                Image, p("mono_cam_topic"), self.mono_callback,
                qos_profile_sensor_data, callback_group=self.image_cb_group)

        if self.imu_used:
            # Deep best-effort queue: bursts of 200-400 Hz IMU accumulate
            # while a frame is being tracked; a shallow queue would drop them
            # and starve the preintegration window.
            imu_qos = QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                history=HistoryPolicy.KEEP_LAST,
                depth=2000,
            )
            self.create_subscription(
                Imu, p("imu_topic"), self.imu_callback, imu_qos,
                callback_group=self.imu_cb_group)

        self.get_logger().info(
            f"Subscribed; waiting for data ({self.camera_setup}"
            f"{'+imu' if self.imu_used else ''})"
        )

    # -- IMU ----------------------------------------------------------------

    def imu_callback(self, msg: Imu):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        row = (
            t,
            msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z,
            msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z,
        )
        with self.imu_lock:
            self.imu_buffer.append(row)

    def _newest_imu_time(self):
        with self.imu_lock:
            return self.imu_buffer[-1][0] if self.imu_buffer else None

    def _wait_for_imu_coverage(self, t_frame: float) -> bool:
        """Delay tracking until IMU up to the frame time has been buffered.

        The IMU callback runs on another executor thread (and the tracking
        call releases the GIL), so this only stalls the image pipeline.
        """
        if not self.imu_used:
            return True
        deadline = time.monotonic() + self.imu_wait_timeout
        while time.monotonic() < deadline:
            newest = self._newest_imu_time()
            if newest is not None and newest >= t_frame:
                return True
            time.sleep(0.002)
        return False

    def _take_imu_window(self, t_frame: float) -> np.ndarray:
        """Pop all measurements with t <= t_frame -> (N, 7) [t,g,g,g,a,a,a]."""
        rows = []
        with self.imu_lock:
            while self.imu_buffer and self.imu_buffer[0][0] <= t_frame:
                rows.append(self.imu_buffer.popleft())
        if not rows:
            return np.empty((0, 7), dtype=np.float64)
        return np.asarray(rows, dtype=np.float64)

    # -- frame plumbing -------------------------------------------------------

    def _stamp_of(self, msg) -> float:
        return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

    def _to_gray(self, msg: Image) -> np.ndarray:
        return self.bridge.imgmsg_to_cv2(msg, desired_encoding="mono8")

    def _pre_track(self, t: float):
        """Common gate for all frame callbacks. Returns imu array or None to skip."""
        if self.prev_frame_timestamp is not None and t <= self.prev_frame_timestamp:
            self.get_logger().error(
                f"Frame timestamp went backward ({self.prev_frame_timestamp:.3f} -> "
                f"{t:.3f}); flushing IMU. Replaying a bag with --loop is not supported."
            )
            with self.imu_lock:
                self.imu_buffer.clear()
            self.prev_frame_timestamp = None
            return None

        if self.imu_used:
            if not self._wait_for_imu_coverage(t):
                self.frames_skipped += 1
                self.get_logger().warn(
                    f"No IMU coverage up to frame t={t:.3f} within "
                    f"{self.imu_wait_timeout}s; skipping frame", throttle_duration_sec=2.0)
                return None
            imu = self._take_imu_window(t)
            if imu.shape[0] == 0 and self.prev_frame_timestamp is not None:
                self.frames_skipped += 1
                with self.imu_lock:
                    oldest = self.imu_buffer[0][0] if self.imu_buffer else float("nan")
                    newest = self.imu_buffer[-1][0] if self.imu_buffer else float("nan")
                    n = len(self.imu_buffer)
                self.get_logger().warn(
                    f"Empty IMU window for frame t={t:.3f}; skipping frame "
                    f"(buffer n={n} oldest-t={oldest - t:+.3f}s newest-t={newest - t:+.3f}s)",
                    throttle_duration_sec=2.0)
                return None
            return imu
        return np.empty((0, 7), dtype=np.float64)

    def _post_track(self, result, t: float):
        self.prev_frame_timestamp = t
        self.frames += 1
        if result.ok:
            self.frames_ok += 1

        state = int(result.state)
        if state != self.last_state:
            self.get_logger().info(f"Tracking state -> {result.state.name}")
            self.last_state = state
        msg = Int32()
        msg.data = state
        self.state_pub.publish(msg)

        if self.imu_used and not self.imu_init_logged and self.slam.imu_initialized():
            self.imu_init_logged = True
            self.get_logger().info("IMU initialized (metric scale + gravity available)")

        if result.pose_valid:
            self._publish_pose(result.pose_wc, t)

        # periodic status line
        self._status_frames += 1
        now = time.monotonic()
        dt = now - self._status_wall
        if dt >= float(self.get_parameter("status_period").value):
            self.get_logger().info(
                f"[status] state={result.state.name} fps={self._status_frames / dt:.1f} "
                f"ok={self.frames_ok}/{self.frames} skipped={self.frames_skipped} "
                f"tracked_pts={result.num_tracked_points}"
                + (f" imu_init={self.slam.imu_initialized()}" if self.imu_used else "")
            )
            self._status_wall = now
            self._status_frames = 0

    def _publish_pose(self, Twc: np.ndarray, t: float):
        x, y, z, w = _rotation_to_quaternion(Twc[:3, :3])
        px, py, pz = Twc[:3, 3]

        stamp_sec = int(t)
        stamp_nsec = int(round((t - stamp_sec) * 1e9))

        odom = Odometry()
        odom.header.frame_id = self.odom_frame
        odom.header.stamp.sec = stamp_sec
        odom.header.stamp.nanosec = stamp_nsec
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = float(px)
        odom.pose.pose.position.y = float(py)
        odom.pose.pose.position.z = float(pz)
        odom.pose.pose.orientation.x = float(x)
        odom.pose.pose.orientation.y = float(y)
        odom.pose.pose.orientation.z = float(z)
        odom.pose.pose.orientation.w = float(w)
        self.odom_pub.publish(odom)

        pose = PoseStamped()
        pose.header = odom.header
        pose.pose = odom.pose.pose
        self.pose_pub.publish(pose)

    # -- frame callbacks ------------------------------------------------------

    def mono_callback(self, msg: Image):
        try:
            t = self._stamp_of(msg)
            imu = self._pre_track(t)
            if imu is None:
                return
            result = self.slam.track_monocular(self._to_gray(msg), t, imu=imu)
            self._post_track(result, t)
        except Exception as e:  # never let one bad frame kill the executor
            self.get_logger().error(f"mono track failed: {e}")

    def stereo_callback(self, left_msg: Image, right_msg: Image):
        try:
            t = self._stamp_of(left_msg)
            imu = self._pre_track(t)
            if imu is None:
                return
            result = self.slam.track_stereo(
                self._to_gray(left_msg), self._to_gray(right_msg), t, imu=imu)
            self._post_track(result, t)
        except Exception as e:
            self.get_logger().error(f"stereo track failed: {e}")

    def rgbd_callback(self, rgb_msg: Image, depth_msg: Image):
        try:
            t = self._stamp_of(rgb_msg)
            imu = self._pre_track(t)
            if imu is None:
                return
            depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")
            depth = np.ascontiguousarray(depth, dtype=np.float32)
            result = self.slam.track_rgbd(self._to_gray(rgb_msg), depth, t, imu=imu)
            self._post_track(result, t)
        except Exception as e:
            self.get_logger().error(f"rgbd track failed: {e}")

    # -- lifecycle ------------------------------------------------------------

    def destroy_node(self):
        try:
            self.get_logger().info(
                f"Shutting down SLAM ({self.frames_ok}/{self.frames} frames OK, "
                f"{self.frames_skipped} skipped)")
            self.slam.shutdown()
            if self.save_trajectory_path:
                self.slam.save_trajectory_euroc(self.save_trajectory_path)
                self.slam.save_keyframe_trajectory_euroc(
                    self.save_trajectory_path + ".kf")
                self.get_logger().info(
                    f"Trajectory saved to {self.save_trajectory_path}")
        except Exception as e:
            self.get_logger().error(f"shutdown/save failed: {e}")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = OrbSlamNode()
    # Multi-threaded so the IMU callback keeps running while tracking blocks.
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
