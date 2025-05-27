import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Header
from geometry_msgs.msg import Pose, Point, Quaternion
import cv2
import numpy as np

class MapPublisher(Node):
    def __init__(self):
        super().__init__('map_publisher')

        self.publisher = self.create_publisher(OccupancyGrid, 'map', 10)

        # Load your processed binary map
        img = cv2.imread('src/image_processing/image_processing/GenerateMap/track_cleaned.png', cv2.IMREAD_GRAYSCALE)
        if img is None:
            self.get_logger().error("Image not found")
            return

        # Invert colors: white = free space (0), black = occupied (100)
        # You may need to threshold if image is not clean binary
        _, binary = cv2.threshold(img, 127, 255, cv2.THRESH_BINARY)

        # Map metadata
        self.map_msg = OccupancyGrid()
        self.map_msg.header = Header()
        self.map_msg.header.frame_id = 'map'  # fixed frame

        self.map_msg.info.resolution = 0.00128388  # meters per pixel (adjust to your scale!)
        self.map_msg.info.width = binary.shape[1]
        self.map_msg.info.height = binary.shape[0]
        self.map_msg.info.origin = Pose()
        self.map_msg.info.origin.position = Point(x=0.0, y=0.0, z=0.0)
        self.map_msg.info.origin.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)

        # Convert image pixels to OccupancyGrid data (-1 unknown, 0 free, 100 occupied)
        data = []
        for pixel in binary.flatten():
            if pixel == 255:
                data.append(0)  # free space
            else:
                data.append(100)  # occupied

        self.map_msg.data = data

        # Publish at 1 Hz
        timer_period = 1.0
        self.timer = self.create_timer(timer_period, self.publish_map)

    def publish_map(self):
        self.map_msg.header.stamp = self.get_clock().now().to_msg()
        self.publisher.publish(self.map_msg)
        self.get_logger().info('Published OccupancyGrid map')

def main(args=None):
    rclpy.init(args=args)
    node = MapPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
