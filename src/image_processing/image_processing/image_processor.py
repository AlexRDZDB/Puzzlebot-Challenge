'''
ROS2 Node to process incoming images and send them to corresponding topic
'''
import cv2
import numpy as np

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class ImageProcessor(Node):
    
    def __init__(self):
        super().__init__("image_processor")

        # Create publishers to send to /traffic_img and /line_img
        self.traffic_img_pub = self.create_publisher(Image, 'camera/traffic_img', 10)
        self.line_img_pub = self.create_publisher(Image, 'camera/line_img', 10)

        # Create subscriber to receive image publisher
        self.img_subscriber = self.create_subscription(Image, 'camera/image_raw', self.img_callback, 10)
        
        # CvBridge object
        self.bridge = CvBridge()

        # Variable to save current frame 
        self.curr_frame = None

        # Timers for sending the processed images according to specified frequency in Hz
        self.declare_parameter("line_freq", 5.0)
        self.declare_parameter("traffic_freq", 10.0)

        self.line_freq = self.get_parameter('line_freq').get_parameter_value().double_value
        self.traffic_freq = self.get_parameter('traffic_freq').get_parameter_value().double_value

        self.line_timer = self.create_timer(1 / self.line_freq, self.line_callback)
        self.traffic_timer = self.create_timer(1/ self.traffic_freq, self.traffic_callback)


    def line_callback(self):
        pass

    def traffic_callback(self):
        # Implement a color filter to filter out everything but shades of green, red and yellow
        
        # Define color ranges
        lower_red1 = np.array([0, 100, 100])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([160, 100, 100])
        upper_red2 = np.array([180, 255, 255])

        lower_green = np.array([40, 70, 70])
        upper_green = np.array([90, 255, 255])

        lower_yellow = np.array([20, 100, 100])
        upper_yellow = np.array([35, 255, 255])

        try:
            hsv_img = cv2.cvtColor(self.curr_frame, cv2.COLOR_BGR2HSV)

            # Create masks
            mask_red = cv2.inRange(hsv_img, lower_red1, upper_red1) | cv2.inRange(hsv_img, lower_red2, upper_red2)
            mask_yellow = cv2.inRange(hsv_img, lower_yellow, upper_yellow)
            mask_green = cv2.inRange(hsv_img, lower_green, upper_green)

            # Combine masks
            combined_mask = cv2.bitwise_or(mask_red, mask_yellow)
            combined_mask = cv2.bitwise_or(combined_mask, mask_green)

            # Erode Mask
            kernel = np.ones((5,5), np.uint8)

            eroded_mask = cv2.erode(combined_mask, kernel, iterations=3)

            dilated_mask = cv2.dilate(eroded_mask, kernel, iterations=3)

            # Find contours from the binary mask
            contours, _ = cv2.findContours(dilated_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            # Create a new blank mask
            filled_mask = np.zeros_like(eroded_mask)

            # Fill each contour
            cv2.drawContours(filled_mask, contours, -1, 255, thickness=cv2.FILLED)

            # Apply mask to original image
            result = cv2.bitwise_and(self.curr_frame, self.curr_frame, mask=filled_mask)

            # Publish result to traffic topic
            msg = self.bridge.cv2_to_imgmsg(result, encoding="bgr8")
            self.traffic_img_pub.publish(msg)

        except Exception as e:
            self.get_logger().info(f"Couldn't process image: {e}")
        pass

    def img_callback(self, msg):
        try:
            self.curr_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().info(f"Could not convert image: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = ImageProcessor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
