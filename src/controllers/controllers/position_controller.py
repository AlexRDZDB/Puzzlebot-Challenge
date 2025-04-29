'''
PID Controller Node for Point to Point Navigation

Usage: Send a Vector3 message containing the setpoint and desired position to the speed controller node via pos_signal topic
Subscribe to the /odom topic to obtain Robot position
Subscribe to the /curr_setpoint to obtain robot's current target position

'''

import math

import tf2_ros
import rclpy
from rclpy import qos
from rclpy.node import Node
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from nav_msgs.msg import Odometry

class PIDController(Node):
    def __init__(self):
        super().__init__("position_controller")

        # Subscriber to /odom topic
        self.odometry_subscriber = self.create_subscription(Odometry, 'odom', self.odometry_callback, qos.qos_profile_sensor_data)

        # Subscriber to /setpoint topic
        self.setpoint_subscriber = self.create_subscription(Vector3, 'curr_setpoint', self.setpoint_callback, 10)

        # Publisher to /cmd_vel topic
        self.pos_singal_publisher = self.create_publisher(Twist, 'pos_signal', 10)

        # Initial Setpoint Values
        self.setpoint_x = 0.0
        self.setpoint_y = 0.0

        # Declare parameters for controlling maximum speeds -> Ranges from 0% to 100%
        self.declare_parameter('MAX_V', 1.0)
        self.declare_parameter('MAX_W', 1.0)

        # Declare parameters for control constants
        self.declare_parameter('KpPos', 2.5)
        self.declare_parameter('KdPos', 0.1) # For adjusting linear velocities
        self.declare_parameter('KiPos', 0.1)

        self.declare_parameter('KpHead', 5.0)
        self.declare_parameter('KiHead', 0.001) # For adjusting angular velocities
        self.declare_parameter('KdHead', 0.1)

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

        # Message Success
        self.get_logger().info("Controller Node started")

    def odometry_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        yaw = q.z
        
        ang = self.normalizeAngle(yaw)

        # Calc difference in sample time
        curr_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        dt = curr_time - self.prev_sample_time
        self.prev_sample_time = curr_time
        
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
        if error_pos <= 0.05:
            v = 0.0
            w = 0.0
            self.integral_pos = 0.0  # Optionally clear integrators
            self.integral_ang = 0.0
        
        # Publish to robot
        cmd = Twist()
        cmd.linear.x = max(min(v, self.MAX_V), -self.MAX_V)
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
    controller = PIDController()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()