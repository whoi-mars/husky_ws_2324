#! /usr/bin/env python3

# Convert pointcloud obstacle readout to list of penguins

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import PoseStamped,Point
from local_nav_pkg.msg import Penguin, PenguinApproachStatus, PenguinInferenceList, PenguinList
from builtin_interfaces.msg import Duration
import math

PENGUIN_DETECTION_ON = False
DETECT_RESEARCHERS = True

class ObstaclePublisherNode(Node):

    def __init__(self):
        super().__init__('target_obstacle_publisher')
    
        self.point_cloud_subscriber = self.create_subscription(MarkerArray, 'visualization_marker_array', self.marker_array_callback, 10)
        self.inferece_subscriber = self.create_subscription(PenguinInferenceList, 'inference',self.inference_callback, 10)
        self.penguin_list_subscriber = self.create_subscription(PenguinList, 'penguin_list',self.penguin_list_callback,10)
        self.status_subscriber = self.create_subscription(PenguinApproachStatus, 'penguin_approach_status', self.approach_status_callback, 10)
        
        self.penguin_publisher_ = self.create_publisher(PenguinList, 'penguin_list', 10)
        self.target_publisher_ = self.create_publisher(Marker, 'target_object', 10)
        self.image_obj_publisher_ = self.create_publisher(Marker, 'image_object', 10)
        self.goal_pose_publisher_ = self.create_publisher(PoseStamped, 'local_goal_pose', 10)
        self.approach_status_publisher_ = self.create_publisher(PenguinApproachStatus, 'penguin_approach_status', 10)
        
        self.x_goal = 0.0
        self.y_goal = 0.0

        self.current_penguin_label = str(0)
        self.approach_status = "pending"
        self.visited_penguins = []
        self.reset_counter = 0

        self.lidar_points = []
        self.existing_penguin_list = []
        self.penguin_label_count = 1
        self.penguin_inferences = []

        self.scale = [0.0, 0.0, 0.5, 0.5, 0.5, 1.0]
    
    # Create a list of points as read from the obstacle detector
    def marker_array_callback(self, msg):       
        self.lidar_points = []
        for i in range(len(msg.markers)):
            point = []
            point.append(msg.markers[i].pose.position.x)
            point.append(msg.markers[i].pose.position.y)
            point.append(msg.markers[i].pose.position.z)
            point.append(msg.markers[i].scale.x)
            point.append(msg.markers[i].scale.y)
            point.append(msg.markers[i].scale.z)
            self.lidar_points.append(point)
        self.publish_penguins()
              
    # Read list of existing penguins from last run through this script
    def penguin_list_callback(self, msg):
        existing_penguin_list = []
        for i in range(len(msg.penguins)):
            existing_penguin_list.append(msg.penguins[i])
        self.existing_penguin_list = existing_penguin_list

    def inference_to_angle_transform(self, box):
        buffer_size = 2 #degrees
        angle1 = ((box.corner1[0])/100)*360 - 180 - buffer_size
        angle2 = ((box.corner3[0])/100)*360 - 180 + buffer_size
        return round(angle1,3), round(angle2,3)
    
    def inference_callback(self, msg):     
        self.penguin_inferences = []
        for i in range(len(msg.penguin_inference)):
            if DETECT_RESEARCHERS:
                if msg.penguin_inference[i].type == 3: #researcher
                    penguin_angle = self.inference_to_angle_transform(msg.penguin_inference[i].box)
                    self.penguin_inferences.append(penguin_angle)
            else:
                if msg.penguin_inference[i].type == 0: #adult standing penguin
                    penguin_angle = self.inference_to_angle_transform(msg.penguin_inference[i].box)
                    self.penguin_inferences.append(penguin_angle)

    # Read approach status, if completed on the last round, move back to zero state and set status to pending
    def approach_status_callback(self, msg):
        self.current_penguin_label = msg.label
        self.approach_status = msg.status

        if self.approach_status == "complete":
            if self.current_penguin_label not in self.visited_penguins:
                self.visited_penguins.append(self.current_penguin_label)
            self.current_penguin_label = str(0)
            self.approach_status = "pending"
        
            penguin_status = PenguinApproachStatus()
            penguin_status.label = self.current_penguin_label
            penguin_status.status = self.approach_status

            self.approach_status_publisher_.publish(penguin_status)     

    def publish_penguins(self):     
        final_penguin_list = PenguinList()
        accumulated_penguin_list = []
        accumulated_labels = []
        unfound_penguins = self.existing_penguin_list.copy()
        #print('existing len: ', len(self.existing_penguin_list))
        for i in range(len(self.lidar_points)):
            already_exists = False
            # iterate through previously detected penguins
            max_duplicate_dist = 0.75
            duplicate_found = False 
            for j in range(len(self.existing_penguin_list)):
                dist = math.sqrt((self.existing_penguin_list[j].point.x - self.lidar_points[i][0])**2 + (self.existing_penguin_list[j].point.y - self.lidar_points[i][1])**2)
                
                if dist < max_duplicate_dist:
                    duplicate_found = True
                    max_duplicate_dist = dist
                    #print('Penguin Exists')
                    old_penguin = Penguin()
                    old_penguin.age = 0
                    old_penguin.point.x = self.lidar_points[i][0]
                    old_penguin.point.y = self.lidar_points[i][1]
                    old_penguin.point.z = self.lidar_points[i][2]
                    old_penguin.scale.x = self.lidar_points[i][3]
                    old_penguin.scale.y = self.lidar_points[i][4]
                    old_penguin.scale.z = self.lidar_points[i][5]
                    old_penguin.label = self.existing_penguin_list[j].label
                    old_penguin.visited = self.existing_penguin_list[j].visited
                    match = self.existing_penguin_list[j]
                    
            if duplicate_found:
                # print(accumulated_penguin_list)
                # print(old_penguin)
                if old_penguin.label not in accumulated_labels:
                    # print(old_penguin.label)
                    accumulated_penguin_list.append(old_penguin)
                    accumulated_labels.append(old_penguin.label)
                    if match in unfound_penguins:
                        unfound_penguins.remove(match)
                    already_exists = True
                # else:
                #     print('GOT EM')
            if not already_exists:
                #print('Penguin Does Not Exist')
                new_penguin = Penguin()
                new_penguin.age = 0
                new_penguin.point.x = self.lidar_points[i][0]
                new_penguin.point.y = self.lidar_points[i][1]
                new_penguin.point.z = self.lidar_points[i][2]
                new_penguin.scale.x = self.lidar_points[i][3]
                new_penguin.scale.y = self.lidar_points[i][4]
                new_penguin.scale.z = self.lidar_points[i][5]
                new_penguin.label = str(self.penguin_label_count)
                new_penguin.visited = "unvisited"
                self.penguin_label_count += 1
                if new_penguin not in accumulated_penguin_list:
                    accumulated_penguin_list.append(new_penguin)
        #print('length of old list', len(test))
        for peng in unfound_penguins:
            peng.age += 1    
            if peng.age < 100:
                accumulated_penguin_list.append(peng)

        sorted_penguin_list = sorted(accumulated_penguin_list, key=lambda penguin: penguin.label)
        self.scale = [0.0, 0.0, 0.5, 0.5, 0.5, 1.0]
        distance = 100
        
        if self.approach_status == "pending":
            for i in range(len(sorted_penguin_list)):
                if sorted_penguin_list[i].label not in self.visited_penguins and sorted_penguin_list[i].visited != 'completed':
                    i_distance = (sorted_penguin_list[i].point.x)**2 + (sorted_penguin_list[i].point.y)**2
                    if sorted_penguin_list[i].point.x > 2: # only take goals in front of robot
                        if i_distance > 1.0: # do not take goals withing 2m of robot - mostly for mobility and so it wont target me during testing
                            if i_distance < distance:
                                if PENGUIN_DETECTION_ON:
                                    for angles in self.penguin_inferences:
                                        #print('Detection Ranges: ', angles[0], angles[1])
                                        if angles[0] < -math.atan2(sorted_penguin_list[i].point.y, sorted_penguin_list[i].point.x)*57.29 < angles[1]:
                                            print("target acquired")
                                            self.current_penguin_label = sorted_penguin_list[i].label
                                            self.x_goal = sorted_penguin_list[i].point.x - sorted_penguin_list[i].scale.x/2
                                            self.y_goal = sorted_penguin_list[i].point.y #- sorted_penguin_list[i].scale.y/2
                                            self.scale = [sorted_penguin_list[i].point.x, sorted_penguin_list[i].point.y, sorted_penguin_list[i].point.z, sorted_penguin_list[i].scale.x,sorted_penguin_list[i].scale.y,sorted_penguin_list[i].scale.z]
                                            distance = i_distance
                                            self.approach_status = "engaging"
                                                                               
                                else:
                                    self.current_penguin_label = sorted_penguin_list[i].label
                                    self.x_goal = sorted_penguin_list[i].point.x - sorted_penguin_list[i].scale.x/2
                                    self.y_goal = sorted_penguin_list[i].point.y #- sorted_penguin_list[i].scale.y/2
                                    self.scale = [sorted_penguin_list[i].point.x, sorted_penguin_list[i].point.y, sorted_penguin_list[i].point.z, sorted_penguin_list[i].scale.x,sorted_penguin_list[i].scale.y,sorted_penguin_list[i].scale.z]
                                    distance = i_distance
                                    self.approach_status = "engaging"
            for penguin in sorted_penguin_list:
                if penguin.label == self.current_penguin_label:
                    penguin.visited = "current"
        else:
            goal_updated = False
            for i in range(len(sorted_penguin_list)):
                if sorted_penguin_list[i].label == self.current_penguin_label:
                    self.x_goal = sorted_penguin_list[i].point.x - sorted_penguin_list[i].scale.x/2
                    self.y_goal = sorted_penguin_list[i].point.y #- sorted_penguin_list[i].scale.y/2
                    self.scale = [sorted_penguin_list[i].point.x, sorted_penguin_list[i].point.y, sorted_penguin_list[i].point.z, sorted_penguin_list[i].scale.x,sorted_penguin_list[i].scale.y,sorted_penguin_list[i].scale.z]
                    goal_updated = True
                    self.reset_counter = 0                    
                    # print("goal updated")
            if not goal_updated:
                self.reset_counter += 1
        
        if self.approach_status == "disengaging":
            for penguin in sorted_penguin_list:
                if penguin.label == self.current_penguin_label:
                    penguin.visited = "completed"

        if self.reset_counter > 40:
            for penguin in sorted_penguin_list:
                if penguin.label == self.current_penguin_label:
                    penguin.visited = "unvisited"
            self.reset_counter = 0
            self.x_goal = 0.0
            self.y_goal = 0.0
            self.approach_status = "pending"
            self.current_penguin_label = str(0)
            print("goal lost -- resetting")
               
        # print('Goal Position: ', self.x_goal, self.y_goal)
        
        final_penguin_list.penguins = sorted_penguin_list
        self.penguin_publisher_.publish(final_penguin_list)
        self.publish_goal_pose()
        self.publish_target()
        self.publish_image_obj()
        self.publish_approach_status()
  
    def publish_goal_pose(self):
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'velodyne'
        goal_pose.header.stamp.sec = 0
        goal_pose.pose.position.x = self.x_goal
        goal_pose.pose.position.y = self.y_goal
        goal_pose.pose.position.z = 0.0
        self.goal_pose_publisher_.publish(goal_pose)

    def publish_target(self):
        target = Marker()
        target.header.frame_id = "velodyne"
        target.header.stamp.sec = 0
        target.ns = "target"
        target.id = 1
        target.type = 1
        lifetime = Duration()
        lifetime.nanosec = 200000000
        target.lifetime = lifetime
        target.pose.position.x = self.scale[0]
        target.pose.position.y = self.scale[1]
        target.pose.position.z = self.scale[2]
        
        target.scale.x = self.scale[3]
        target.scale.y = self.scale[4]
        target.scale.z = self.scale[5]
        target.color.r = 1.0
        target.color.g = 0.0
        target.color.b = 0.0
        target.color.a = 0.5
        self.target_publisher_.publish(target) 

    def publish_image_obj(self):
        image_obj = Marker()
        image_obj.header.frame_id = "base_link"
        image_obj.header.stamp.sec = 0
        image_obj.ns = "image"
        image_obj.id = 1
        image_obj.type = 5
            
        angle_viz_list = []
        angle_viz_distance = 10

        for angles in self.penguin_inferences:
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

    def publish_approach_status(self):
        penguin_status = PenguinApproachStatus()
        penguin_status.label = self.current_penguin_label
        penguin_status.status = self.approach_status

        self.approach_status_publisher_.publish(penguin_status)

def main(args=None):  
    rclpy.init(args=args)
    print('Start')
    node = ObstaclePublisherNode()
    rclpy.spin(node)
    rclpy.shutdown()
  
if __name__ == '__main__':
  main()
