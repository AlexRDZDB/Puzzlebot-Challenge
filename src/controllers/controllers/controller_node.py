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
        super().__init__("Position_Controller")

        # Subscriber to /odom topic
        self.odometry_subscriber = self.create_subscription(Vector3, 'odom', self.odometry_callback, 10)

        # Subscriber to /setpoint topic
        self.setpoint_subscriber = self.create_subscription(Vector3, 'subscription', self.setpoint_callback, 10)

        # Publisher to /cmd_vel topic
        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vek', 10)

        # Current Setpoint Values
        self.setpoint_x = 0.0
        self.setpoint_y = 0.0

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
        ang = self.normalizeAngle(msg.z)

        # Calc difference in sample time
        curr_time = self.get_clock().now().nanoseconds * 1e-9
        dt = curr_time - self.prev_sample_time

        # Calculate position erros
        error_x = self.setpoint_x - x 
        error_y = self.setpoint_y - y

        # Use a distance error to calculate needed velocity
        error_pos = math.sqrt(error_x**2 + error_y**2)
        error_d_pos = (error_pos - self.prev_error_pos) / dt
        self.integral_pos += error_pos * dt
        
        # Control input for linear velocity
        v = error_pos * self.kp_pos + error_d_pos * self.kd_pos + self.integral_pos * self.ki_pos

        # Use heading to calculate necessary angular veolcity
        target_ang = math.atan2(error_y, error_x)

        error_ang = self.normalizeAngle(target_ang - ang)
        error_d_ang = self.normalizeAngle((error_ang - self.prev_error_ang) / dt)
        self.integral_ang += error_ang * dt

        # Control input for angular velocity
        w = error_ang * self.kp_head + error_d_ang * self.kd_head + self.integral_ang * self.ki_head

        # Stop the robot if it's in an acceptable position
        if error_pos <= 0.01:
            v = 0.0
            w = 0.0
        
        # Publish to robot
        cmd = Twist()
        cmd.linear.x = max(min(v, self.MAX_V), -self.MIN_V)
        cmd.angular.z = max(min(w, self.MAX_W), -self.MAX_W)
        
        self.cmd_vel_publisher.publish(cmd)

        # Update necessary variables for next calculation
        self.prev_error_pos = error_pos
        self.prev_error_ang = error_ang
    
    # Updates setpoints
    def setpoint_callback(self, msg):
        self.setpoint_x = msg.x
        self.setpoint_y = msg.y

    # Function for normalizing angles to the range of -π to π
    def normalizeAngle(self, angle):
        return (angle + math.pi) % (2 * math.pi) - math.pi
    
def main(args=None):
    rclpy.init(args=args)
    controller = positionController()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()