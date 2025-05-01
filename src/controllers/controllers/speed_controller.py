'''
Controller for modifying final robot output speeds and publishing to the actual robot
'''

import math

import tf2_ros
import rclpy
from rclpy import qos
from rclpy.node import Node
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from nav_msgs.msg import Odometry


class SpeedController(Node):
    def __init__(self):
        super().__init__("speed_controller")

        # Subscriber to /pos_signal topic
        self.pos_signal_subscriber = self.create_subscription(Twist, 'pos_signal', self.pos_signal_callback, 10)

        # Publisher to /cmd_vel
        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
    
    def pos_signal_callback(self, msg):
        v = msg.linear.x
        w = msg.angular.z

        cmd = Twist()
        cmd.linear.x = v
        cmd.angular.z = w

        self.cmd_vel_publisher.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    controller = SpeedController()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()