#!/usr/bin/env python3
"""SDK-less ZED 2i ROS2 driver node.

Images are captured over UVC/V4L2 with OpenCV (side-by-side stereo, split in
half); inertial/environmental sensors come from zed-open-capture via the
zed_sensors Python wrapper (https://github.com/NaiveInvestigator/zed-open-capture-python).

Fixes over the previous version of this node:
  * IMU messages now use the sensor's hardware timestamps for their relative
    spacing (clean ~3.4 ms deltas) instead of publish-time wall clock, which
    removed the polling jitter that visual-inertial SLAM chokes on. The
    hardware clock's absolute epoch is NOT host time when zed-open-capture's
    own video capture isn't running (observed ~2 minutes ahead), so the node
    continuously re-bases it onto the host clock with a slowly adapting
    offset — keeping IMU stamps on the same clock as the image stamps.
  * The ZED reports IMU at 400 Hz; the old 200 Hz polling timer silently
    dropped half the samples and re-published duplicates. A dedicated polling
    thread with timestamp dedup now publishes every distinct sample.
  * Image grabbing runs in its own thread — cap.read() blocks ~33 ms, which
    used to stall the whole single-threaded executor (IMU included).
  * /dev/video* index is a parameter, with auto-detection by device name
    ("ZED") as the default instead of a hard-coded index.
  * All topics and frame ids are parameters with defaults.

Gyro is reported by the library in deg/s and converted to rad/s per REP-145;
accelerometer is already m/s^2; magnetometer comes in uT and is converted to
Tesla.
"""

from __future__ import annotations

import glob
import math
import threading

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import (
    FluidPressure,
    Image,
    Imu,
    MagneticField,
    RelativeHumidity,
    Temperature,
)

from zed_sensors import ZedSensors


def find_zed_video_device() -> int | None:
    """Return the lowest /dev/videoN index whose driver name contains 'ZED'.

    The ZED exposes two video nodes; the even/lower one is the capture node.
    """
    candidates = []
    for path in sorted(glob.glob("/sys/class/video4linux/video*/name")):
        try:
            with open(path) as f:
                name = f.read().strip()
        except OSError:
            continue
        if "zed" in name.lower():
            idx = int(path.split("video")[-1].split("/")[0])
            candidates.append(idx)
    return min(candidates) if candidates else None


def _ns_to_stamp(ns: int, msg_stamp) -> None:
    msg_stamp.sec = int(ns // 1_000_000_000)
    msg_stamp.nanosec = int(ns % 1_000_000_000)


class ZedStereoNode(Node):
    def __init__(self):
        super().__init__("zed_stereo_ros")
        self.bridge = CvBridge()

        # -- parameters -------------------------------------------------------
        self.declare_parameter("video_device", -1)  # -1 = auto-detect by name
        # Side-by-side stream size: 1344x376 = VGA per eye, 2560x720 = HD720.
        self.declare_parameter("frame_width", 1344)
        self.declare_parameter("frame_height", 376)
        self.declare_parameter("fps", 30.0)
        # Constant offset added to image stamps (s) to compensate capture
        # latency relative to the IMU hardware clock; tune if needed.
        self.declare_parameter("image_time_offset", 0.0)

        self.declare_parameter("left_image_topic", "/zed/left/image_raw")
        self.declare_parameter("right_image_topic", "/zed/right/image_raw")
        self.declare_parameter("imu_topic", "/zed/imu/data")
        self.declare_parameter("mag_topic", "/zed/imu/mag")
        self.declare_parameter("publish_environment", True)
        self.declare_parameter("env_temperature_topic", "/zed/env/temperature")
        self.declare_parameter("env_pressure_topic", "/zed/env/pressure")
        self.declare_parameter("env_humidity_topic", "/zed/env/humidity")
        self.declare_parameter("left_temperature_topic", "/zed/temperature/left")
        self.declare_parameter("right_temperature_topic", "/zed/temperature/right")

        self.declare_parameter("left_frame_id", "zed_left_link")
        self.declare_parameter("right_frame_id", "zed_right_link")
        self.declare_parameter("imu_frame_id", "zed_imu_link")

        p = lambda name: self.get_parameter(name).value  # noqa: E731

        self.left_frame_id = p("left_frame_id")
        self.right_frame_id = p("right_frame_id")
        self.imu_frame_id = p("imu_frame_id")
        self.image_time_offset = float(p("image_time_offset"))

        # -- publishers ---------------------------------------------------------
        self.left_pub = self.create_publisher(
            Image, p("left_image_topic"), qos_profile_sensor_data)
        self.right_pub = self.create_publisher(
            Image, p("right_image_topic"), qos_profile_sensor_data)
        self.imu_pub = self.create_publisher(Imu, p("imu_topic"), qos_profile_sensor_data)
        self.mag_pub = self.create_publisher(
            MagneticField, p("mag_topic"), qos_profile_sensor_data)

        self.publish_environment = bool(p("publish_environment"))
        if self.publish_environment:
            self.env_temp_pub = self.create_publisher(
                Temperature, p("env_temperature_topic"), 10)
            self.pressure_pub = self.create_publisher(
                FluidPressure, p("env_pressure_topic"), 10)
            self.humidity_pub = self.create_publisher(
                RelativeHumidity, p("env_humidity_topic"), 10)
            self.cam_temp_left_pub = self.create_publisher(
                Temperature, p("left_temperature_topic"), 10)
            self.cam_temp_right_pub = self.create_publisher(
                Temperature, p("right_temperature_topic"), 10)

        # -- camera ------------------------------------------------------------
        device = int(p("video_device"))
        if device < 0:
            detected = find_zed_video_device()
            if detected is None:
                raise RuntimeError(
                    "No ZED video device found (checked /sys/class/video4linux). "
                    "Set the video_device parameter explicitly.")
            device = detected
            self.get_logger().info(f"Auto-detected ZED at /dev/video{device}")

        self._device = device
        self._width = int(p("frame_width"))
        self._height = int(p("frame_height"))
        self._fps = float(p("fps"))
        self.cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
        if not self.cap.isOpened():
            raise RuntimeError(
                f"Failed to open /dev/video{device}. Check permissions (video group).")
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self._width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self._height)
        self.cap.set(cv2.CAP_PROP_FPS, self._fps)
        # Smallest V4L2 queue -> the frame we stamp with now() really is fresh.
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        w = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        h = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        fps = self.cap.get(cv2.CAP_PROP_FPS)
        self.get_logger().info(f"Camera stream: {w}x{h} (side-by-side) @ {fps:.0f} fps")

        # -- sensors -------------------------------------------------------------
        self.sens = ZedSensors(verbosity="error")
        self.sensors_ready = bool(self.sens.initialize_sensors(-1))
        if self.sensors_ready:
            sn = self.sens.get_serial_number()
            fw = self.sens.get_firmware_version()
            self.get_logger().info(f"Sensors ready: SN={sn} FW={fw[0]}.{fw[1]}")
        else:
            self.get_logger().error(
                "ZED sensors failed to initialize; publishing images only.")

        # -- worker threads ---------------------------------------------------
        self._clock_offset_ns = None  # sensor-clock -> host-clock rebase
        self._running = True
        self._threads = [threading.Thread(target=self._image_loop, daemon=True)]
        if self.sensors_ready:
            self._threads.append(threading.Thread(target=self._imu_loop, daemon=True))
            self.create_timer(0.1, self._slow_sensor_callback)
        for t in self._threads:
            t.start()

        self.get_logger().info("ZED stereo node started")

    # -- image capture (thread) ---------------------------------------------

    def _image_loop(self):
        dropped = 0
        while self._running and rclpy.ok():
            ret, frame = self.cap.read()
            stamp = self.get_clock().now().to_msg()
            if not ret:
                dropped += 1
                self.get_logger().warn(
                    f"Camera frame grab failed (x{dropped}); still trying")
                if dropped >= 1:  # each failed read is a 10 s select() timeout
                    # A killed reader can leave the UVC stream wedged
                    # (select() timeouts forever); reopening recovers it.
                    self.get_logger().warn("Reopening video device...")
                    self.cap.release()
                    self.cap = cv2.VideoCapture(self._device, cv2.CAP_V4L2)
                    self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self._width)
                    self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self._height)
                    self.cap.set(cv2.CAP_PROP_FPS, self._fps)
                    self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
                    dropped = 0
                continue
            dropped = 0

            if self.image_time_offset:
                total = stamp.sec + stamp.nanosec * 1e-9 + self.image_time_offset
                stamp.sec = int(total)
                stamp.nanosec = int(round((total - int(total)) * 1e9))

            half = frame.shape[1] // 2
            left_msg = self.bridge.cv2_to_imgmsg(frame[:, :half], encoding="bgr8")
            right_msg = self.bridge.cv2_to_imgmsg(frame[:, half:], encoding="bgr8")
            left_msg.header.stamp = stamp
            left_msg.header.frame_id = self.left_frame_id
            right_msg.header.stamp = stamp
            right_msg.header.frame_id = self.right_frame_id
            try:
                self.left_pub.publish(left_msg)
                self.right_pub.publish(right_msg)
            except Exception:
                if not (self._running and rclpy.ok()):
                    break  # context torn down mid-publish; normal at shutdown
                raise

    # -- IMU + magnetometer (thread, ~400 Hz) --------------------------------

    def _rebase_to_host_ns(self, sensor_ts_ns: int) -> int:
        """Map a sensor-clock timestamp onto the host clock.

        offset = host_now - sensor_ts, tracked with a slow EMA so the sensor
        clock's clean relative spacing is preserved while absolute time stays
        anchored to the host (the sensor epoch is arbitrary without the
        zed-open-capture video sync). Re-anchors on jumps > 0.5 s (device
        re-enumeration).
        """
        now_ns = self.get_clock().now().nanoseconds
        inst = now_ns - sensor_ts_ns
        if self._clock_offset_ns is None or abs(inst - self._clock_offset_ns) > 500_000_000:
            self._clock_offset_ns = inst
        else:
            self._clock_offset_ns += int(0.001 * (inst - self._clock_offset_ns))
        return sensor_ts_ns + self._clock_offset_ns

    def _imu_loop(self):
        last_imu_ts = None
        last_mag_ts = None
        while self._running and rclpy.ok():
            # Blocks in C++ (GIL released by pybind) until fresh data or timeout.
            d = self.sens.get_last_imu_data(timeout_usec=5000)
            if d["valid"] and d["timestamp"] != last_imu_ts:
                last_imu_ts = d["timestamp"]
                msg = Imu()
                _ns_to_stamp(self._rebase_to_host_ns(d["timestamp"]), msg.header.stamp)
                msg.header.frame_id = self.imu_frame_id
                msg.linear_acceleration.x = float(d["accel"][0])
                msg.linear_acceleration.y = float(d["accel"][1])
                msg.linear_acceleration.z = float(d["accel"][2])
                # library reports deg/s -> rad/s
                msg.angular_velocity.x = math.radians(float(d["gyro"][0]))
                msg.angular_velocity.y = math.radians(float(d["gyro"][1]))
                msg.angular_velocity.z = math.radians(float(d["gyro"][2]))
                # No fused orientation available (REP-145)
                msg.orientation_covariance[0] = -1.0
                try:
                    self.imu_pub.publish(msg)
                except Exception:
                    if not (self._running and rclpy.ok()):
                        break
                    raise

            mag = self.sens.get_last_magnetometer_data(timeout_usec=10)
            if mag["valid"] and mag["timestamp"] != last_mag_ts:
                last_mag_ts = mag["timestamp"]
                m = MagneticField()
                _ns_to_stamp(self._rebase_to_host_ns(mag["timestamp"]), m.header.stamp)
                m.header.frame_id = self.imu_frame_id
                m.magnetic_field.x = float(mag["field"][0]) * 1e-6  # uT -> T
                m.magnetic_field.y = float(mag["field"][1]) * 1e-6
                m.magnetic_field.z = float(mag["field"][2]) * 1e-6
                self.mag_pub.publish(m)

    # -- environment (timer, 10 Hz) -------------------------------------------

    def _slow_sensor_callback(self):
        if not self.publish_environment:
            return
        env = self.sens.get_last_environment_data(timeout_usec=100)
        if env["valid"]:
            t = Temperature()
            _ns_to_stamp(self._rebase_to_host_ns(env["timestamp"]), t.header.stamp)
            t.header.frame_id = self.imu_frame_id
            t.temperature = float(env["temperature"])
            self.env_temp_pub.publish(t)

            pr = FluidPressure()
            pr.header = t.header
            pr.fluid_pressure = float(env["pressure"]) * 100.0  # hPa -> Pa
            self.pressure_pub.publish(pr)

            h = RelativeHumidity()
            h.header = t.header
            h.relative_humidity = float(env["humidity"]) / 100.0
            self.humidity_pub.publish(h)

        temp = self.sens.get_last_camera_temperature_data(timeout_usec=100)
        if temp["valid"]:
            tl = Temperature()
            _ns_to_stamp(self._rebase_to_host_ns(temp["timestamp"]), tl.header.stamp)
            tl.header.frame_id = self.left_frame_id
            tl.temperature = float(temp["temp_left"])
            self.cam_temp_left_pub.publish(tl)

            tr = Temperature()
            tr.header = tl.header
            tr.header.frame_id = self.right_frame_id
            tr.temperature = float(temp["temp_right"])
            self.cam_temp_right_pub.publish(tr)

    # -- lifecycle -------------------------------------------------------------

    def destroy_node(self):
        self._running = False
        for t in self._threads:
            t.join(timeout=2.0)
        if self.cap is not None:
            self.cap.release()
        super().destroy_node()


def main(args=None):
    from rclpy.executors import ExternalShutdownException

    rclpy.init(args=args)
    node = ZedStereoNode()
    try:
        rclpy.spin(node)
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
