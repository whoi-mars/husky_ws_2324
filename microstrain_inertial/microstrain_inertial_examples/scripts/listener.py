#! /usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

class Listener(Node):
    def __init__(self):
        super().__init__("listener")
        self.subscription = self.create_subscription(Imu, '/imu/data', self.imuDataCallback, 10)

    def imuDataCallback(self, imu):
        self.get_logger().info("Quaternion Orientation:  [%f, %f, %f, %f]" % (imu.orientation.x, imu.orientation.y, imu.orientation.z, imu.orientation.w));
        self.get_logger().info("Angular Velocity:        [%f, %f, %f]" % (imu.angular_velocity.x, imu.angular_velocity.y, imu.angular_velocity.z));
        # add code here to handle incoming IMU data
    
if __name__ == '__main__':
    rclpy.init(args=None)

    listener = Listener()

    rclpy.spin(listener)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    listener.destroy_node()
    rclpy.shutdown()
