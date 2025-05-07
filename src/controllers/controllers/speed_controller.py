'''
Controller for modifying final robot output speeds and publishing to the actual robot
'''

import math

import tf2_ros
import rclpy
from rclpy import qos
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from nav_msgs.msg import Odometry


class SpeedController(Node):
    def __init__(self):
        super().__init__("speed_controller")

        # Subscriber to /pos_signal topic
        self.pos_signal_subscriber = self.create_subscription(Twist, 'pos_signal', self.pos_signal_callback, 10)

        # Subscriber to /traffic_signal topic
        self.traffic_signal_sub = self.create_subscription(String, 'traffic_signal', self.traffic_callback, 10)
        self.curr_traffic_modifier = 1.0

        # Publisher to /cmd_vel
        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)

    def pos_signal_callback(self, msg):
        v = msg.linear.x * self.curr_traffic_modifier
        w = msg.angular.z * self.curr_traffic_modifier

        cmd = Twist()
        cmd.linear.x = v
        cmd.angular.z = w

        self.get_logger().info(f"Linear: {v}, Angular {w}")
        self.cmd_vel_publisher.publish(cmd)

    def traffic_callback(self, msg):
        light = msg.data

        if light == 'red':
            self.curr_traffic_modifier = 0.0
        elif light == 'yellow':
            self.curr_traffic_modifier = 0.5
        elif light == 'green':
            self.curr_traffic_modifier = 1.0
        
def main(args=None):
    rclpy.init(args=args)
    controller = SpeedController()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()