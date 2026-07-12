import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, Imu
from builtin_interfaces.msg import Time
from cv_bridge import CvBridge
import cv2
import os
import time

class TumViPump(Node):
    def __init__(self, dataset_path):
        super().__init__('tum_vi_pump')
        
        # INCREASED QUEUE SIZES: Prevent dropped messages during high load
        self.cam0_pub = self.create_publisher(Image, '/camera/left/image_raw', 50)
        self.cam1_pub = self.create_publisher(Image, '/camera/right/image_raw', 50)
        self.imu_pub = self.create_publisher(Imu, '/camera/imu', 1000) 
        
        self.bridge = CvBridge()
        self.dataset_path = dataset_path
        self.events = []
        
        self.get_logger().info("Parsing CSV files and loading timeline...")
        self.load_imu()
        self.load_camera('cam0')
        self.load_camera('cam1')
        
        self.events.sort(key=lambda x: x[0])
        self.get_logger().info(f"Loaded {len(self.events)} total sensor events. Starting data pump...")

    # ... [load_camera and load_imu remain exactly the same as your code] ...
    def load_camera(self, cam_name):
        csv_path = os.path.join(self.dataset_path, cam_name, 'data.csv')
        img_dir = os.path.join(self.dataset_path, cam_name, 'data')
        if not os.path.exists(csv_path): return
        with open(csv_path, 'r') as f:
            for line in f:
                if line.startswith('#'): continue
                parts = line.strip().split(',')
                if len(parts) >= 2:
                    self.events.append((int(parts[0]), cam_name, os.path.join(img_dir, parts[1])))

    def load_imu(self):
        csv_path = os.path.join(self.dataset_path, 'imu0', 'data.csv')
        if not os.path.exists(csv_path): return
        with open(csv_path, 'r') as f:
            for line in f:
                if line.startswith('#'): continue
                parts = line.strip().split(',')
                if len(parts) >= 7:
                    self.events.append((int(parts[0]), 'imu', parts))

    def stream(self):
        if not self.events:
            return
            
        start_time_ns = self.events[0][0]
        start_time_real = time.time()
        
        for ts, event_type, data in self.events:
            if not rclpy.ok():
                break
            
            # 1. PREPARE THE MESSAGE FIRST (Do disk I/O before we wait)
            msg = None
            ros_time = Time()
            ros_time.sec = int(ts // 1e9)
            ros_time.nanosec = int(ts % 1e9)
            
            if event_type == 'imu':
                msg = Imu()
                msg.header.stamp = ros_time
                msg.header.frame_id = 'imu_link'
                msg.angular_velocity.x = float(data[1])
                msg.angular_velocity.y = float(data[2])
                msg.angular_velocity.z = float(data[3])
                msg.linear_acceleration.x = float(data[4])
                msg.linear_acceleration.y = float(data[5])
                msg.linear_acceleration.z = float(data[6])
                
            elif event_type in ['cam0', 'cam1']:
                cv_img = cv2.imread(data, cv2.IMREAD_GRAYSCALE)
                if cv_img is not None:
                    msg = self.bridge.cv2_to_imgmsg(cv_img, encoding="mono8")
                    msg.header.stamp = ros_time
                    msg.header.frame_id = event_type + '_link'

            # 2. HIGH-PRECISION WAIT
            elapsed_sim_time = (ts - start_time_ns) / 1e9
            target_real_time = start_time_real + elapsed_sim_time
            sleep_time = target_real_time - time.time()
            
            # Hybrid sleep: standard sleep for the bulk, busy-spin for the last 2ms
            if sleep_time > 0.002:
                time.sleep(sleep_time - 0.002)
            while time.time() < target_real_time:
                pass  # Spin to hit the exact microsecond
                
            # 3. PUBLISH INSTANTLY
            if msg is not None:
                if event_type == 'imu':
                    self.imu_pub.publish(msg)
                elif event_type == 'cam0':
                    self.cam0_pub.publish(msg)
                elif event_type == 'cam1':
                    self.cam1_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    dataset_dir = '/home/mahir/orbulating/dataset/tum/dataset-room1_512_16/mav0'
    pump = TumViPump(dataset_dir)
    try:
        pump.stream()
    except KeyboardInterrupt:
        pass
    finally:
        pump.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()