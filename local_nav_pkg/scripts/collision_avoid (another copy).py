#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from visualization_msgs.msg import MarkerArray
from local_nav_pkg.msg import PenguinList

class CollisionAvoidanceVelocityPublisherNode(Node):

    def __init__(self):
        super().__init__('collision_avoidance_velocity_publisher')
        self.subscriber = self.create_subscription(Twist, 'approach_vel', self.current_velocity_callback, 10)
        self.point_cloud_subscriber = self.create_subscription(MarkerArray, 'visualization_marker_array', self.marker_array_callback, 10)
        self.penguin_list_subscriber = self.create_subscription(PenguinList, 'penguin_list', self.penguin_list_callback, 10)
        self.goal_pose_subscriber = self.create_subscription(PoseStamped, 'local_goal_pose', self.goal_pose_callback, 10)
        
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        #self.collision_timer_ = self.create_timer(0.1, self.publish_collision_avoidance)

        self.speed_linear_x = 0.0
        self.speed_angular_z= 0.0
        self.list_of_points = []
        self.penguin_points = []
        self.goal_pose_x = 0.0
        self.goal_pose_y = 0.0
        
    def goal_pose_callback(self, msg):
        self.goal_pose_x = msg.pose.position.x
        self.goal_pose_y = msg.pose.position.y
        
    def current_velocity_callback(self, msg):
        self.speed_linear_x = msg.linear.x
        self.speed_angular_z = msg.angular.z

        self.publish_collision_avoidance()

    def marker_array_callback(self, msg):
        self.list_of_points = []
        for i in range(len(msg.markers)):
            point = []
            point.append(msg.markers[i].pose.position.x)
            point.append(msg.markers[i].pose.position.y)
            point.append(msg.markers[i].pose.position.z)
            point.append(msg.markers[i].scale.x)
            point.append(msg.markers[i].scale.y)
            point.append(msg.markers[i].scale.z)
            self.list_of_points.append(point)

    def penguin_list_callback(self, msg):
        self.penguin_points = []
        for i in range(len(msg.penguins)):
            point = []
            point.append(msg.penguins[i].point.x)
            point.append(msg.penguins[i].point.y)
            point.append(msg.penguins[i].point.z)
            point.append(msg.penguins[i].scale.x)
            point.append(msg.penguins[i].scale.y)
            point.append(msg.penguins[i].scale.z)
            self.penguin_points.append(point)

    def publish_collision_avoidance(self):
        collision_avoid_vel = Twist()
        new_speed_linear_x = self.speed_linear_x
        new_speed_angular_z = self.speed_angular_z
        new_list = self.list_of_points + self.penguin_points

        closest_collision_back = 1000
        for point in new_list:
            if point[0] > 0:
                closest_point = point[0]-point[3]/2
                if self.speed_linear_x > 0:
                    if closest_point < .75 and abs(point[1]) < .75: #if there is a point right in front of the robot - stop.
                        print('forwards collision imminent - stopping')
                        new_speed_linear_x = 0.0
                        new_speed_angular_z = 0.0
                       
                    
                    elif closest_point < 2.5 and abs(point[1]) < .75 and closest_point - self.goal_pose_x > 1.0: #if point is far, but in route, try turning
                        print('forward collision detected - rerouting')
                        arrival_time = self.get_clock().now().nanoseconds        
                        wait_time = 10 * 10**9
                        print('turning')
                        while self.get_clock().now().nanoseconds < arrival_time + wait_time:
                            
                            if point[1] > 0:
                                new_speed_linear_x = .1
                                new_speed_angular_z = .1333*3.14
                            else:
                                new_speed_linear_x = .1
                                new_speed_angular_z = -.1333*3.14
                            collision_avoid_vel.linear.x = new_speed_linear_x
                            collision_avoid_vel.angular.z = new_speed_angular_z
                            self.publisher_.publish(collision_avoid_vel)
                        while self.get_clock().now().nanoseconds < arrival_time + wait_time*2:
                            new_speed_linear_x = 0.1
                            new_speed_angular_z = 0.0
                            collision_avoid_vel.linear.x = new_speed_linear_x
                            collision_avoid_vel.angular.z = new_speed_angular_z
                            self.publisher_.publish(collision_avoid_vel)
            else:
                closest_point = point[0] + point[3]/2
                if self.speed_linear_x < 0:
                    if closest_point > -.75 and abs(point[1]) < .75:
                        print('backwards collision imminent - stopping')
                        new_speed_linear_x = 0.0
                        new_speed_angular_z = 0.0
                        
                    elif closest_point > -2.5 and abs(point[1]) < .75:
                        print('backwards collision detected - rerouting')
                        arrival_time = self.get_clock().now().nanoseconds        
                        wait_time = 2 * 10**9
                        print('turning')
                        while self.get_clock().now().nanoseconds < arrival_time + wait_time:
                            if point[1] > 0:
                                new_speed_linear_x = -.1
                                new_speed_angular_z = -.1333*3.14
                            else:
                                new_speed_linear_x = -.1
                                new_speed_angular_z = .1333*3.14
                            collision_avoid_vel.linear.x = new_speed_linear_x
                            collision_avoid_vel.angular.z = new_speed_angular_z
                            self.publisher_.publish(collision_avoid_vel)
                        while self.get_clock().now().nanoseconds < arrival_time + wait_time*2:
                            new_speed_linear_x = -0.1
                            new_speed_angular_z = 0.0
                            collision_avoid_vel.linear.x = new_speed_linear_x
                            collision_avoid_vel.angular.z = new_speed_angular_z
                            self.publisher_.publish(collision_avoid_vel)
                            
        
        collision_avoid_vel.linear.x = new_speed_linear_x
        collision_avoid_vel.angular.z = new_speed_angular_z
        self.publisher_.publish(collision_avoid_vel)
         
def main(args=None):  
    rclpy.init(args=args)
    node = CollisionAvoidanceVelocityPublisherNode()
    rclpy.spin(node)
    rclpy.shutdown()
 
  
if __name__ == '__main__':
  main()
