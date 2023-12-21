#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
import serial
from rclpy.node import Node
import utm
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Header

class GPS_Publisher(Node):

    def __init__(self):
        super().__init__('gps_publisher')
        self.publisher_ = self.create_publisher(NavSatFix, 'gps/data', 10)

        #Sensor parameter configuration
        SENSOR_NAME = "gps"
        serial_port = '/dev/gps_port'
        serial_baud = 4800
        port = serial.Serial(serial_port, serial_baud, timeout=3.)
        self.get_logger().info("Initialization complete")

        msg = NavSatFix()
        i = 0
        while i != -1:
            line = port.readline()
            # print(line)
            if line == '':
                self.get_logger().info("Not reading any data from sensor")
                # rospy.logwarn("Not reading any data from sensor")
            else:
                if line.startswith(b'$GPGGA'):
                    print("\nFound GPGGA in line")
                    gpgga_array = line.split(b',')
                    print(gpgga_array)
                    header = gpgga_array[0]
                    latitude = gpgga_array[2].decode('utf-8')
                    print("Latitude is:",latitude)
                    if latitude != '':
                        latitude_direction = gpgga_array[3].decode('utf-8')
                        latitude_dd = self.dms_to_dd(float(latitude[:2]) , float(latitude[2:9]), 0)
                        latitude_dd  = latitude_dd if latitude_direction == "N" else latitude_dd*-1
                        longitude = gpgga_array[4].decode('utf-8')
                        longitude_direction = gpgga_array[5].decode('utf-8')
                        longitude_dd = self.dms_to_dd(float(longitude[:3]) , float(longitude[3:10]) , 0)                    
                        longitude_dd  = longitude_dd if longitude_direction == "E" else longitude_dd*-1
                        altitude = float(gpgga_array[9].decode('utf-8'))
                        altitude_unit = gpgga_array[10].decode('utf-8')
                        msg.header.stamp = Node.get_clock(self).now().to_msg()
                        msg.header.frame_id = "base_link"
                        msg.latitude = latitude_dd
                        msg.longitude = longitude_dd
                        msg.altitude = altitude
                        self.publisher_.publish(msg)
                        self.get_logger().info('Publishing: "%s"' % msg)

    # Function to convert Degree Minute Second (DMS) to Decimal Degree(DD) notation
    def dms_to_dd(self, d, min, sec):
        dd = d + float(min)/60 + float(sec)/3600
        return dd

def main(args=None):
    rclpy.init(args=args)
    gps_publisher = GPS_Publisher()
    rclpy.spin(gps_publisher)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    gps_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
