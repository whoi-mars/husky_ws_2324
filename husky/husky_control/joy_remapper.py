#!/usr/bin/env python3
# -*- coding: utf-8 -*-

#from matplotlib.backend_bases import LocationEvent

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Header

class Joy_Publisher(Node):

    def __init__(self):
        super().__init__('joy_publisher')
        self.publisher_ = self.create_publisher(Joy, '/joy_teleop/joy', 10)
        timer_period = 1/150  # seconds
        # self.timer = self.create_timer(timer_period, self.timer_callback)

        self.joy_subscription = self.create_subscription(Joy,'/joy_remapped',self.joy_callback,10)
        self.joy_subscription  # prevent unused variable warning

    def joy_callback(self, data):
        #print(data)



        
        
        
        msg = Joy()
        msg.header.stamp = Node.get_clock(self).now().to_msg()
        msg.header.frame_id = "joy"
        data.axes[1] = -data.axes[1]
        msg.axes = data.axes
        msg.buttons = data.buttons
        #msg.buttons = buttons=[0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0]

        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg)

      

def main(args=None):
    rclpy.init(args=args)

    joy_publisher = Joy_Publisher()

    rclpy.spin(joy_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    joy_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
