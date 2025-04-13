import rclpy
import numpy as np
import signal

from rclpy import qos
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import Vector3


class DeadReckoning(Node):

    def __init__(self):
        super().__init__('dead_reckoning')

        # Parameters
        self.X = 0.0
        self.Y = 0.0
        self.Th = 0.0
        self._l = 0.18
        self._r = 0.05
        self._sample_time = 0.01
        self.rate = 200.0
                    
        self.first = True
        self.start_time = 0.0
        self.current_time = 0.0
        self.last_time = 0.0

        self.v_r = 0.0
        self.v_l = 0.0
        self.V = 0.0
        self.Omega = 0.0

        self.wr = Float32()
        self.wl = Float32()
        self.vec_msg = Vector3()

        # Subscriptions
        self.sub_encR = self.create_subscription(Float32, 'VelocityEncR', self.encR_callback, qos.qos_profile_sensor_data)
        self.sub_encL = self.create_subscription(Float32, 'VelocityEncL', self.encL_callback, qos.qos_profile_sensor_data)

        # Publisher (now a Vector3)
        self.odom_vec_pub = self.create_publisher(Vector3, 'odom_vec', qos.qos_profile_sensor_data)

        # Timer
        self.timer = self.create_timer(1.0 / self.rate, self.run)

        self.get_logger().info("Dead Reckoning Node Started (Vector3 Output)")

    def encR_callback(self, msg):
        self.wr = msg

    def encL_callback(self, msg):
        self.wl = msg

    def run(self):
        if self.first:
            self.start_time = self.get_clock().now()
            self.last_time = self.start_time
            self.current_time = self.start_time
            self.first = False
            return
        
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds * 1e-9
        
        if dt > self._sample_time:
            # Tangential velocities
            self.v_r = self._r * self.wr.data
            self.v_l = self._r * self.wl.data

            # Robot velocities
            self.V = 0.5 * (self.v_r + self.v_l)
            self.Omega = (1.0 / self._l) * (self.v_r - self.v_l)

            # Integrate position and angle
            self.X += self.V * np.cos(self.Th) * dt
            self.Y += self.V * np.sin(self.Th) * dt
            self.Th += self.Omega * dt

            self.last_time = current_time

            self.publish_odometry_vec()

    def publish_odometry_vec(self):
        """ Publish Vector3 with X, Y, Theta """
        self.vec_msg.x = self.X
        self.vec_msg.y = self.Y
        self.vec_msg.z = self.Th
        self.odom_vec_pub.publish(self.vec_msg)

    def stop_handler(self, signum, frame):
        self.get_logger().info("Interrupt received! Shutting down node.")
        raise SystemExit


def main(args=None):
    rclpy.init(args=args)
    node = DeadReckoning()
    signal.signal(signal.SIGINT, node.stop_handler)

    try:
        rclpy.spin(node)
    except SystemExit:
        node.get_logger().info('SystemExit triggered. Node stopped.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
