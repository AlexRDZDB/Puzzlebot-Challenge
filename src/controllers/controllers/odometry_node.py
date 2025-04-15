'''
    Robot localization Node
    Subscribes to /VelocityEncR and /VelocityEncL to recieve encoder values
    Interprets encoder values and transforms them to robot position
'''

import rclpy
import transforms3d
import math
import signal, os, time

from rclpy import qos
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster

class RobotLocalization(Node):
    def __init__(self):
        super().__init__('localization-node')

        # Publisher to Odom topic of type odometry
        self.odometry_publisher = self.create_publisher(Odometry, 'odom', qos.qos_profile_sensor_data)

        # Subscriber to encoder values
        self.left_velocity_subscriber = self.create_subscription(Float32, 'VelocityEncL', self.update_wl, qos.qos_profile_sensor_data)
        self.right_velocity_subscriber = self.create_subscription(Float32, 'VelocityEncR', self.update_wr, qos.qos_profile_sensor_data)

        # Store angular velocity values in rad/s
        self.wl = 0.0
        self.wr = 0.0 

        # Store Robot localization parameters
        self.X = 0.0 # meters
        self.Y = 0.0 # meters
        self.Theta = 0.0 # rads

        # Robot configuration parameters (Intended for a two-wheel differential drive robot)
        self.declare_parameter('wheel_distance', 0.18) # meters
        self.declare_parameter('wheel_radius', 0.05) # meters

        self.wheel_distance = self.get_parameter('wheel_distance').get_parameter_value().double_value
        self.wheel_radius = self.get_parameter('wheel_radius').get_parameter_value().double_value

        # Sampling time parameters
        self.declare_parameter('sample_time', 0.01) # seconds per sample
        self.declare_parameter('rate', 200.0) # Hz

        self.sample_time = self.get_parameter('sample_time').get_parameter_value().double_value
        self.rate = self.get_parameter('rate').get_parameter_value().double_value

        # Internal state of the robot
        self.start_time = self.get_clock().now().nanoseconds * 1e-9
        self.curr_time = self.start_time
        self.last_time = self.start_time

        # Timer to calculate odometry data according to the sampling rate
        self.timer = self.create_timer(1.0 / self.rate, self.calc_odometry)

        # Logger for recognizing start of localization
        self.get_logger().info("Localization Started")

    # Functions to update velocity values according to encoder data
    def update_wl(self, msg):
        self.wl = msg.data
    
    def update_wr(self, msg):
        self.wr = msg.data

    # Function to update and publish odometry values
    def calc_odometry(self):
        
        # Obtain new current time and dt
        self.curr_time = self.get_clock().now().nanoseconds * 1e-9
        dt = self.curr_time - self.last_time

        # Ensure timeframe is acceptable
        if dt > self.sample_time:

            # Calculate tangential velocities
            vr = self.wr * self.wheel_radius
            vl = self.wl * self.wheel_radius

            # Calculate robot velocity
            V = 0.5 * (vr + vl)

            # Calculate robot angular velocity
            W = (vr - vl) / self.wheel_distance

            # Helper function to normalize the theta angle between -pi and pi
            def normalizeAngle(angle):
                return (angle + math.pi) % (2 * math.pi) - math.pi
            
            # Update X, Y and Theta values
            self.X += math.cos(self.Theta) * V * dt
            self.Y += math.sin(self.Theta) * V * dt
            self.Theta += W * dt
            self.Theta = normalizeAngle(self.Theta)

            # Create odometry message to publish
            odom_msg = Odometry()

            # Send position information
            odom_msg.pose.pose.position.x = self.X
            odom_msg.pose.pose.position.y = self.Y
            odom_msg.pose.pose.position.z = 0.0

            # Send angle information
            q1 = transforms3d.euler.euler2quat(0,0, self.Theta) # Transform to quaternions to upload to message
            odom_msg.pose.orientation.x = q1[1]
            odom_msg.pose.orientation.y = q1[2]
            odom_msg.pose.orientation.z = q1[3]
            odom_msg.pose.orientation.w = q1[0]

            # Send velocities
            odom_msg.twist.twist.linear.x = V
            odom_msg.twist.twist.angular.z = W

            # Send header information
            odom_msg.header.stamp = self.get_clock().now().to_msg()
            odom_msg.header.frame_id = 'odom'
            odom_msg.child_frame_id = "base_footprint"

            # Publish message to /odom topic
            self.odometry_publisher.publish(odom_msg)
            self.last_time = self.curr_time

    # Function for handling node shutdown
    def stop_handler(self, signum, frame):
        self.get_logger().info('SIGINT received. Shutting down...')
        rclpy.shutdown()


def main(args=None):

    rclpy.init(args=args)

    node = RobotLocalization()

    signal.signal(signal.SIGINT, node.stop_handler)

    try:
        rclpy.spin(node)
    except SystemExit:
        node.get_logger().info('SystemExit triggered. Shutting down cleanly.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()