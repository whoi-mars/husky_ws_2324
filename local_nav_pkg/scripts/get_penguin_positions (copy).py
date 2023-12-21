#! /usr/bin/env python3

# Convert pointcloud obstacle readout to list of penguins

import rclpy
#import rospy
from rclpy.node import Node
from visualization_msgs.msg import MarkerArray
from visualization_msgs.msg import Marker

from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point


from local_nav_pkg.msg import Penguin
from local_nav_pkg.msg import PenguinList
from local_nav_pkg.msg import PenguinApproachStatus
from local_nav_pkg.msg import PenguinInferenceList

import math

list_of_points = []
penguin_list_raw = []
penguin_label_count = 1
penguin_inferences = []

PENGUIN_DETECTION_ON = True

#x = 0.0
#y = 0.0
x_goal = 0.0
y_goal = 0.0

odometry_pose_x = 0.0
odometry_pose_y = 0.0

current_penguin_label = str(0)
approach_status = "pending"
visited_penguins = []
reset_counter = 0

scale = [0.0, 0.0, 0.5, 0.5, 0.5, 1.0]

class ObstaclePublisherNode(Node):

    def __init__(self):
        super().__init__('target_obstacle_publisher')
    
        self.odometry_subscriber = self.create_subscription(Odometry, 'odom', self.odometry_callback, 10)
        self.point_cloud_subscriber = self.create_subscription(MarkerArray, 'visualization_marker_array', self.marker_array_callback, 10)
        self.inferece_subscriber = self.create_subscription(PenguinInferenceList, 'inference',self.inference_callback, 10)
        self.penguin_list_subscriber = self.create_subscription(PenguinList, 'penguin_list',self.penguin_list_callback,10)
        self.status_subscriber = self.create_subscription(PenguinApproachStatus, 'penguin_approach_status', self.approach_status_callback, 10)
        
        self.penguin_publisher_ = self.create_publisher(PenguinList, 'penguin_list', 10)
        self.target_publisher_ = self.create_publisher(Marker, 'target_object', 10)
        self.image_obj_publisher_ = self.create_publisher(Marker, 'image_object', 10)
        self.penguin_timer_ = self.create_timer(0.1, self.publish_penguins)
        self.goal_pose_publisher_ = self.create_publisher(PoseStamped, 'local_goal_pose', 10)
        #self.goal_pose_timer_ = self.create_timer(0.1, self.publish_goal_pose)
        self.approach_status_publisher_ = self.create_publisher(PenguinApproachStatus, 'penguin_approach_status', 10)
        #self.approach_status_timer_ = self.create_timer(0.1, self.publish_approach_status)
    
    # Read current position
    def odometry_callback(self, msg):
        global odometry_pose_x
        global odometry_pose_y
        odometry_pose_x = msg.pose.pose.position.x
        odometry_pose_y = msg.pose.pose.position.y
    
    
    # Create a list of points as read from the obstacle detector
    def marker_array_callback(self, msg):
        global list_of_points
        list_of_points = []
        for i in range(len(msg.markers)):
            point = []
            point.append(msg.markers[i].pose.position.x)
            point.append(msg.markers[i].pose.position.y)
            point.append(msg.markers[i].pose.position.z)
            point.append(msg.markers[i].scale.x)
            point.append(msg.markers[i].scale.y)
            point.append(msg.markers[i].scale.z)
            list_of_points.append(point)
              
    
    # Read list of existing penguins from last run through this script
    def penguin_list_callback(self, msg):
        global penguin_list_raw       
        penguin_list_raw = []
        for i in range(len(msg.penguins)):
            penguin_list_raw.append(msg.penguins[i])

    def inference_to_angle_transform(self, box):
        #print('box: ', box.corner1[0], box.corner3[0])
        buffer_size = 2
        angle1 = ((box.corner1[0])/100)*360 - 180 - buffer_size
        angle2 = ((box.corner3[0])/100)*360 - 180 + buffer_size
        #print('angles: ', angle1, angle2)
        return round(angle1,3), round(angle2,3)
    
    def inference_callback(self, msg):
        global penguin_inferences       
        penguin_inferences = []
        for i in range(len(msg.penguin_inference)):
            #if msg.penguin_inference[i].type == 0: #adult standing penguin
            if msg.penguin_inference[i].type == 3: #researcher
                penguin_angle = self.inference_to_angle_transform(msg.penguin_inference[i].box)
                penguin_inferences.append(penguin_angle)

    # Read approach status, if completed on the last round, move back to zero state and set status to pending
    def approach_status_callback(self, msg):
        global current_penguin_label
        global approach_status
        global visited_penguins

        current_penguin_label = msg.label
        #print('label', current_penguin_label)
        approach_status = msg.status
        #print('approach', approach_status)
        if approach_status == "complete":
            if current_penguin_label not in visited_penguins:
                visited_penguins.append(current_penguin_label)
            current_penguin_label = str(0)
            approach_status = "pending"
        
        penguin_status = PenguinApproachStatus()
        penguin_status.label = current_penguin_label
        penguin_status.status = approach_status

        self.approach_status_publisher_.publish(penguin_status)     

    def publish_penguins(self):     
        #global penguin_list
        global list_of_points
        global penguin_label_count
        global penguin_inferences

        global x_goal
        global y_goal
        global visited_penguins
        global current_penguin_label
        global odometry_pose_x
        global odometry_pose_y
        global approach_status
        global reset_counter
        global scale
        global penguin_list_raw

        final_penguin_list = []
        final_penguin_list = PenguinList()

        accumulated_penguin_list = []
        
        penguin_list=penguin_list_raw
        # iterate through points detected by lidar
        print(len(list_of_points))
        print(len(penguin_list))
        for i in range(len(list_of_points)):
            already_exists = False
            # iterate through previously detected penguins
            max_duplicate_dist = 0.3
            duplicate_found = False 
            for j in range(len(penguin_list)):                  
                dist = math.sqrt((penguin_list[j].point.x - list_of_points[i][0])**2 + (penguin_list[j].point.y - list_of_points[i][1])**2)
                if dist < max_duplicate_dist:
                    duplicate_found = True
                    max_duplicate_dist = dist
                    print('Penguin Exists')
                    old_penguin = Penguin()
                    old_penguin.point.x = list_of_points[i][0]
                    old_penguin.point.y = list_of_points[i][1]
                    old_penguin.point.z = list_of_points[i][2]
                    old_penguin.scale.x = list_of_points[i][3]
                    old_penguin.scale.y = list_of_points[i][4]
                    old_penguin.scale.z = list_of_points[i][5]
                    old_penguin.label = penguin_list[j].label
                if duplicate_found:
                    if old_penguin not in accumulated_penguin_list:
                        accumulated_penguin_list.append(old_penguin)
                        already_exists = True
            if not already_exists:
                print('Penguin Does Not Exist')
                new_penguin = Penguin()
                new_penguin.point.x = list_of_points[i][0]
                new_penguin.point.y = list_of_points[i][1]
                new_penguin.point.z = list_of_points[i][2]
                new_penguin.scale.x = list_of_points[i][3]
                new_penguin.scale.y = list_of_points[i][4]
                new_penguin.scale.z = list_of_points[i][5]
                new_penguin.label = str(penguin_label_count)
                penguin_label_count += 1
                if new_penguin not in accumulated_penguin_list:
                    accumulated_penguin_list.append(new_penguin)
            
        sorted_penguin_list = sorted(accumulated_penguin_list, key=lambda penguin: penguin.label)
        #print(accumulated_penguin_list)
        # pick closest penguin as target
        distance = 100
        if approach_status == "pending":
            for i in range(len(sorted_penguin_list)):
                #print(i, approach_status)
                #print(msg.penguins[i])
                if sorted_penguin_list[i].label not in visited_penguins:
                    #i_distance = (odometry_pose_x - sorted_penguin_list[i].point.x)**2 + (odometry_pose_y - sorted_penguin_list[i].point.y)**2
                    i_distance = (sorted_penguin_list[i].point.x)**2 + (sorted_penguin_list[i].point.y)**2
                    
                    if i_distance > 2.0:
                        if i_distance < distance:
                            #print('angle: ', math.atan2(sorted_penguin_list[i].point.y, sorted_penguin_list[i].point.x)*57.29)
                            if PENGUIN_DETECTION_ON:
                                #print(sorted_penguin_list[i].scale)
                                for angles in penguin_inferences:
                                    #print('Detection Ranges: ', angles[0], angles[1])
                                    if angles[0] < -math.atan2(sorted_penguin_list[i].point.y, sorted_penguin_list[i].point.x)*57.29 < angles[1]:
                                        print("target acquired")
                                        
                                        current_penguin_label = sorted_penguin_list[i].label
                                        x_goal = sorted_penguin_list[i].point.x - sorted_penguin_list[i].scale.x/2
                                        y_goal = sorted_penguin_list[i].point.y #- sorted_penguin_list[i].scale.y/2
                                        scale = [sorted_penguin_list[i].point.x, sorted_penguin_list[i].point.y, sorted_penguin_list[i].point.z, sorted_penguin_list[i].scale.x,sorted_penguin_list[i].scale.y,sorted_penguin_list[i].scale.z]
                                        distance = i_distance
                                        approach_status = "engaging"                                        
                            else:
                                current_penguin_label = sorted_penguin_list[i].label
                                x_goal = sorted_penguin_list[i].point.x - sorted_penguin_list[i].scale.x/2
                                y_goal = sorted_penguin_list[i].point.y #- sorted_penguin_list[i].scale.y/2
                                scale = [sorted_penguin_list[i].point.x, sorted_penguin_list[i].point.y, sorted_penguin_list[i].point.z, sorted_penguin_list[i].scale.x,sorted_penguin_list[i].scale.y,sorted_penguin_list[i].scale.z]
                                distance = i_distance
                                approach_status = "engaging"
        else:
            goal_updated = False
            for i in range(len(sorted_penguin_list)):
                if sorted_penguin_list[i].label == current_penguin_label:
                    x_goal = sorted_penguin_list[i].point.x - sorted_penguin_list[i].scale.x/2
                    y_goal = sorted_penguin_list[i].point.y #- sorted_penguin_list[i].scale.y/2
                    scale = [sorted_penguin_list[i].point.x, sorted_penguin_list[i].point.y, sorted_penguin_list[i].point.z, sorted_penguin_list[i].scale.x,sorted_penguin_list[i].scale.y,sorted_penguin_list[i].scale.z]
                    goal_updated = True
                    reset_counter = 0                    
                    print("goal updated")
            if not goal_updated:
                reset_counter += 1
        #print("reset counter: ",reset_counter)
        if reset_counter > 40:
            reset_counter = 0
            x_goal = 0.0
            y_goal = 0.0
            approach_status = "pending"
            current_penguin_label = str(0)
            print("goal lost -- resetting")
               
        print('Goal Position: ', x_goal, y_goal)
        final_penguin_list.penguins = sorted_penguin_list
        #print(final_penguin_list)    
        self.penguin_publisher_.publish(final_penguin_list)

        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'velodyne'
        goal_pose.header.stamp.sec = 0
        goal_pose.pose.position.x = x_goal
        goal_pose.pose.position.y = y_goal
        goal_pose.pose.position.z = 0.0
        goal_pose.pose.orientation.x = 0.0
        goal_pose.pose.orientation.y = 0.0
        goal_pose.pose.orientation.z = 0.0
        goal_pose.pose.orientation.w = 1.0
        #speed_limit.speed_limit = 1.0
        self.goal_pose_publisher_.publish(goal_pose)

        penguin_status = PenguinApproachStatus()
        penguin_status.label = current_penguin_label
        penguin_status.status = approach_status

        self.approach_status_publisher_.publish(penguin_status)

        target = Marker()
        target.header.frame_id = "base_link"
        target.header.stamp.sec = 0
        target.ns = "target"
        target.id = 1
        target.type = 1
        target.pose.position.x = scale[0]
        target.pose.position.y = scale[1]
        target.pose.position.z = scale[2]
        target.pose.position.z = 0.5
        target.pose.orientation.x = 0.0
        target.pose.orientation.y = 0.0
        target.pose.orientation.z = 0.0
        target.pose.orientation.w = 1.0
        target.scale.x = scale[3]
        target.scale.y = scale[4]
        target.scale.z = scale[5]
        target.color.r = 1.0
        target.color.g = 0.0
        target.color.b = 0.0
        target.color.a = 0.8
        self.target_publisher_.publish(target) 

        image_obj = Marker()
        image_obj.header.frame_id = "base_link"
        image_obj.header.stamp.sec = 0
        image_obj.ns = "image"
        image_obj.id = 1
        image_obj.type = 5
        
        
        angle_viz_list = []
        angle_viz_distance = 10
         

        for angles in penguin_inferences:
            p1 = Point()
            p2 = Point()
            p3 = Point()
            
            p1.x = 0.0
            p1.y = 0.0
            p1.z = 0.0
            
            p2.x = angle_viz_distance* math.cos(-angles[0]/57.29)
            p2.y = angle_viz_distance* math.sin(-angles[0]/57.29)
            p2.z = 0.0
            
            p3.x = angle_viz_distance* math.cos(-angles[1]/57.29)
            p3.y = angle_viz_distance* math.sin(-angles[1]/57.29)
            p3.z = 0.0

            angle_viz_list.append(p1)
            angle_viz_list.append(p2)

            angle_viz_list.append(p2)
            angle_viz_list.append(p3)

            angle_viz_list.append(p3)
            angle_viz_list.append(p1)

            
        
        image_obj.points = angle_viz_list
        image_obj.scale.x = 0.1
        image_obj.pose.orientation.x = 0.0
        image_obj.pose.orientation.y = 0.0
        image_obj.pose.orientation.z = 0.0
        image_obj.pose.orientation.w = 1.0
        
        image_obj.color.r = 1.0
        image_obj.color.g = 0.0
        image_obj.color.b = 0.0
        image_obj.color.a = 1.0
        self.image_obj_publisher_.publish(image_obj) 
        
    def publish_goal_pose(self):     
        global x_goal
        global y_goal
        print(x_goal, y_goal)
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'velodyne'
        goal_pose.header.stamp.sec = 0
        goal_pose.pose.position.x = x_goal
        goal_pose.pose.position.y = y_goal
        goal_pose.pose.position.z = 0.0
        goal_pose.pose.orientation.x = 0.0
        goal_pose.pose.orientation.y = 0.0
        goal_pose.pose.orientation.z = 0.0
        goal_pose.pose.orientation.w = 1.0
        #speed_limit.speed_limit = 1.0
        self.goal_pose_publisher_.publish(goal_pose)
        
    def publish_approach_status(self):
        global current_penguin_label
        global approach_status

        penguin_status = PenguinApproachStatus()

        penguin_status.label = current_penguin_label
        penguin_status.status = approach_status

        self.approach_status_publisher_.publish(penguin_status) 
 
def main(args=None):  
    rclpy.init(args=args)
    print('Start')
    node = ObstaclePublisherNode()
    rclpy.spin(node)
    rclpy.shutdown()
 
  
if __name__ == '__main__':
  main()
