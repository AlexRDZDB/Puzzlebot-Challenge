'''
ROS2 Node to detect the traffic signal present in an image (if applicable)
'''
import cv2
import numpy as np

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class TrafficSignalDetector(Node):
    def __init__(self):
        super().__init__("traffic_signal_detector")

        # Signal publisher 
        self.traffic_signal_pub = self.create_publisher(String, 'traffic_signal', 10)
        
        # Traffic subscription
        self.traffic_subscription = self.create_subscription(Image, 'camera/traffic_img', self.image_callback, 10)

        # Current traffic light
        self.curr_light = "green"

        # CvBridge
        self.bridge = CvBridge()

    # Function to classify the color into red, green or yellow
    def classify_color(self, hsv_pixel):
        h, s, v = hsv_pixel

        # Red (can appear at both low and high H values)
        if (0 <= h <= 10 or 160 <= h <= 180) and s > 100 and v > 50:
            return "red"
        # Yellow
        elif 20 <= h <= 35 and s > 100 and v > 50:
            return "yellow"
        # Green (including darker greens)
        elif 40 <= h <= 90 and s > 60 and v > 40:
            return "green"
        
        return None
            
    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().info(f"Image conversion failed: {e}")
        
        # Convert to grayscale to detect circles
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        gray = cv2.medianBlur(gray, 5)

        # Circle Detection
        circles = cv2.HoughCircles(
            gray,
            cv2.HOUGH_GRADIENT,
            dp = 1.2,
            minDist=20,
            param1=100,
            param2=30,
            minRadius=30,
            maxRadius=240
        )

        if circles is not None:
            circles = np.uint16(np.around(circles[0]))
            best_circle = None
            best_color = None
            max_radius = 0

            for circle in circles:
                x,y,r = circle
                hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
                hsv_pixel = hsv[y,x]
                h,s,v = int(hsv_pixel[0]), int(hsv_pixel[1]), int(hsv_pixel[2])

                detected_color = self.classify_color((h,s,v))


                if detected_color and r > max_radius:
                    max_radius = r
                    best_color = detected_color
                    best_circle = (x,y,r)
            
            if best_circle and self.curr_light != best_color:
                x,y,r = best_circle
                self.get_logger().info(f"Detected traffic Light: {best_circle}")
                
                self.curr_light = best_color
                traffic_msg = String()
                traffic_msg.data = best_color
                self.get_logger().info(f"Current Color = {best_color}")
                self.traffic_signal_pub.publish(traffic_msg)
        
        self.get_logger().info(f"Current color: {self.curr_light}")

def main(args=None):
    rclpy.init(args=args)
    node = TrafficSignalDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()