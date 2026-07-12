import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class ZedNativeNode(Node):
    def __init__(self):
        super().__init__('zed_native_node')
        
        # Initialize Publishers
        self.left_pub = self.create_publisher(Image, '/left_img', 10)
        self.right_pub = self.create_publisher(Image, '/right_img', 10)
        
        self.bridge = CvBridge()
        
        # Open /dev/video2 using OpenCV's V4L2 backend
        self.cap = cv2.VideoCapture(2, cv2.CAP_V4L2)
        
        if not self.cap.isOpened():
            self.get_logger().error('Failed to open /dev/video2. Check device path and permissions.')
            return

        # Optional: Force a specific side-by-side resolution (e.g., 720p side-by-side is 2560x720)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1344)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 376)
        # self.cap.set(cv2.CAP_PROP_FPS, 30)

        # Set up a timer to fetch and publish frames at ~30 Hz
        timer_period = 1.0 / 30.0  
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.get_logger().info('Successfully connected. Publishing ZED 2i images to /left_img and /right_img...')

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn('Frame dropped or camera disconnected.')
            return
            
        # The ZED 2i outputs a side-by-side image over UVC.
        # Slicing the numpy array in half yields the individual camera frames.
        height, width, channels = frame.shape
        half_width = width // 2
        
        left_frame = frame[:, :half_width]
        right_frame = frame[:, half_width:]
        
        # Convert OpenCV BGR images to ROS Image messages
        left_msg = self.bridge.cv2_to_imgmsg(left_frame, encoding='bgr8')
        right_msg = self.bridge.cv2_to_imgmsg(right_frame, encoding='bgr8')
        
        # Attach timestamps and frame IDs
        now = self.get_clock().now().to_msg()
        left_msg.header.stamp = now
        left_msg.header.frame_id = 'zed_left_link'
        
        right_msg.header.stamp = now
        right_msg.header.frame_id = 'zed_right_link'
        
        # Publish to the ROS network
        self.left_pub.publish(left_msg)
        self.right_pub.publish(right_msg)

    def destroy_node(self):
        # Ensure the camera resource is released upon shutdown
        self.cap.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = ZedNativeNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()