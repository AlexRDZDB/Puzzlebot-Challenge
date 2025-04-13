'''
PID Controller Node for Point to Point Navigation

Usage: Send a Vector3 message containing the setpoint and desired position to the robot
Subscribe to the /odom topic to obtain Robot position

'''

import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3

class PIDController(Node):
    def __init__(self):
        
        # Subscriber to /odom topic
        self.odometry_subscriber = self.create_subscription(Vector3, 'odom', self.odometry_callback, 10)

        # Subscriber to /setpoint topic
        self.setpoint_subscriber = self.create_subscription(Vector3, 'subscription', self.setpoint_callback, 10)

        # Publisher to /cmd_vel topic
        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vek', 10)

        # Current Setpoint Values
        self.x = 0.0
        self.y = 0.0

        # Declare parameters for controlling maximum speeds
        self.declare_parameter('MAX_V', 1.0)
        self.declare_parameter('MAX_W', 3.0)

        # Declare parameters for control constants
        self.declare_parameter('KpPos', 1.0)
        self.declare_parameter('KdPos', 0.01) # For adjusting linear velocities
        self.declare_parameter('KiPos', 0.01)

        self.declare_parameter('KpHead', 1.0)
        self.declare_parameter('KiHead', 0.01) # For adjusting angular velocities
        self.declare_parameter('KdHead', 0.01)

        # Controller Variables
        self.prev_error_pos = 0.0
        self.integral_pos = 0.0

        self.prev_error_ang = 0.0
        self.integral_ang = 0.0

        self.prev_sample_time = self.get_clock().now().nanoseconds * 1e-9 # Store in seconds

        # Save parameters locally
        self.MAX_V = self.get_parameter('MAX_V').get_parameter_value().double_value
        self.MAX_W = self.get_parameter('MAX_W').get_parameter_value().double_value

        self.kp_pos = self.get_parameter('KpPos').get_parameter_value().double_value
        self.kd_pos = self.get_parameter('KdPos').get_parameter_value().double_value
        self.ki_pos = self.get_parameter('KiPos').get_parameter_value().double_value

        self.kp_head = self.get_parameter('KpHead').get_parameter_value().double_value
        self.ki_head = self.get_parameter('KiHead').get_parameter_value().double_value
        self.kd_head = self.get_parameter('KdHead').get_parameter_value().double_value

    def odometry_callback(self, msg):
        x = msg.x
        y = msg.y
        theta = msg.z

        curr_time = self.get_clock().now().nanoseconds * 1e-9
        dt = curr_time - self.prev_sample_time
        

    def setpoint_callback(self, msg):
        pass
def main(args=None):
    rclpy.init(args=args)
    controller = positionController()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()