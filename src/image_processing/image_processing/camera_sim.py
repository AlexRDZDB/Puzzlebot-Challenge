'''
Developer node to simulate puzzlebot camera with laptop camera
'''
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class WebcamPublisher(Node):
    def __init__(self):
        super().__init__('webcam_publisher')
        self.publisher_ = self.create_publisher(Image, '/camera/image_raw', 10)
        self.bridge = CvBridge()
        self.cap = cv2.VideoCapture(0)  # Change to desired camera index or video file

        if not self.cap.isOpened():
            self.get_logger().error('❌ Could not open webcam.')
            raise RuntimeError('Webcam not available')

        self.timer = self.create_timer(1.0 / 30.0, self.publish_frame)  # 30 Hz

    def publish_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn('⚠️ Frame capture failed.')
            return

        # Convert BGR OpenCV frame to ROS Image message
        image_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        self.publisher_.publish(image_msg)

    def destroy_node(self):
        self.cap.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = WebcamPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
