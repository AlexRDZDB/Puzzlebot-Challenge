import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from std_msgs.msg import Header
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import PoseStamped


class DifferentialDriveSim(Node):

    def __init__(self):
        super().__init__('differential_drive_sim')
        
        # Simulation state
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        
        # Robot parameters
        self.L = 0.18
        self.r = 0.05

        self.v = 0.0  # linear velocity
        self.w = 0.0  # angular velocity

        self.last_time = self.get_clock().now()

        # Sub to cmd_vel
        self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)

        # Publish to Encoders
        self.wr_publisher = self.create_publisher(Float32, 'VelocityEncR', 10)
        self.wl_publisher = self.create_publisher(Float32, 'VelocityEncL', 10)

        # For visualization/logging only
        self.pose_publisher = self.create_publisher(PoseStamped, 'robot_pose', 10)

        # Update timer
        self.create_timer(0.1, self.update_position)

    def cmd_vel_callback(self, msg):
        self.v = msg.linear.x
        self.w = msg.angular.z

    def update_position(self):
        # Compute elapsed time
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds * 1e-9
        self.last_time = current_time

        if dt == 0:
            return

        # Update position
        delta_x = self.v * math.cos(self.theta) * dt
        delta_y = self.v * math.sin(self.theta) * dt
        delta_theta = self.w * dt

        self.x += delta_x
        self.y += delta_y
        self.theta += delta_theta

        # Publish current simulated velocities
        twist_msg = Twist()
        twist_msg.linear.x = self.v
        twist_msg.angular.z = self.w

        omega_L = (self.v - self.w * (self.L / 2)) / self.r
        omega_R = (self.v + self.w * (self.L / 2)) / self.r
        wr, wl = Float32(), Float32()
        wr.data = omega_R
        wl.data = omega_L
        self.wr_publisher.publish(wr)
        self.wl_publisher.publish(wl)

        # Publish simulated pose (for visualization/debugging)
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'map'  # or 'odom'

        pose_msg.pose.position.x = self.x
        pose_msg.pose.position.y = self.y
        pose_msg.pose.position.z = 0.0

        # Convert theta to quaternion
        qz = math.sin(self.theta / 2.0)
        qw = math.cos(self.theta / 2.0)

        pose_msg.pose.orientation.z = qz
        pose_msg.pose.orientation.w = qw

        self.pose_publisher.publish(pose_msg)

def main(args=None):
    rclpy.init(args=args)
    node = DifferentialDriveSim()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
