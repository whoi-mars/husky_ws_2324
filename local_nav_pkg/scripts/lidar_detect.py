#! /usr/bin/env python3

# Convert raw pointcloud into a list of bounding boxes which approximates the penguins in the scene

import rclpy
import point_cloud2
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import MarkerArray, Marker
from std_msgs.msg import Header
from builtin_interfaces.msg import Duration
import numpy as np
from sklearn.cluster import DBSCAN 

# How far out to look for penguins
RADIUS = 12
#Needed if running on a simulation environment with a perfectly flat ground plane
SIMULATION = False

class LidarDetectionNode(Node):

    def __init__(self):
        super().__init__('lidar_object_publisher')
        self.subscriber = self.create_subscription(PointCloud2, 'velodyne_points', self.velodyne_callback, 1)
        
        self.filtered_cloud_publisher_ = self.create_publisher(PointCloud2, 'filtered_cloud', 1)
        self.marker_publisher_ = self.create_publisher(MarkerArray, 'visualization_marker_array', 1)
        self.zed_obj_subscriber = self.create_subscription(MarkerArray, 'zed_obj_viz_array', self.zed_obj_callback, 10)

        self.cloud_timer_ = self.create_timer(.1, self.cluster)

        self.cloud = [[0.0, 0.0, 0.0]]
        self.zed_markers = MarkerArray()

    # Reading pointcloud
    def velodyne_callback(self, msg):        
        try:
            cloud_raw = point_cloud2.read_points(msg)
            self.cloud = []
            for point in cloud_raw:
                self.cloud.append([point[0],point[1],point[2]])
            
        except IndexError:
            print('no cloud detected yet')

    # Read objects from Zed as well
    def zed_obj_callback(self, msg):
        self.zed_markers.markers = []
        for marker in msg.markers:
            self.zed_markers.markers.append(marker)

    # Seperate the pointcloud into clusters                   
    def cluster(self):
        markers = MarkerArray()    
        self.cloud = np.array(self.cloud)
        #print(cloud)
        # remove all points outside the radius
        mask = np.sqrt(self.cloud[:, 0]**2 + self.cloud[:, 1]**2) < RADIUS
        cropped_cloud = self.cloud[mask]
        if SIMULATION: # if simulation, remove all points that are on the ground - otherwise clustering doesn't work because the data is too perfect
            mask2 = cropped_cloud[:, 2] > -.5
            cropped_cloud = cropped_cloud[mask2]

        # take x y and z columns
        xs0, ys0, zs0 = cropped_cloud[:, 0], cropped_cloud[:, 1], cropped_cloud[:, 2]
        xy_points = cropped_cloud[:, :2]

        # Cluster points using DB scan, ignoring height
        model = DBSCAN(eps=0.075, min_samples=10)
        model_labels = model.fit_predict(xy_points)

        # add cluster labels to the cloud
        mask = model_labels != -1
        new_cloud = np.column_stack((xs0[mask], ys0[mask], zs0[mask], model_labels[mask]))
        xs1, ys1, zs1, labels1 = new_cloud[:, 0], new_cloud[:, 1], new_cloud[:, 2], new_cloud[:, 3]

        xs, ys, zs, labels = [], [], [], []
        
        # Add ZED objects to the list of markers
        for marker in self.zed_markers.markers:
            marker.color.a = 0.5
            markers.markers.append(marker)
        id = 0
        # itereate through all the clusters
        for i in set(labels1):
            marker = Marker()
            marker.id = id
            marker.ns = 'cluster'
            marker.color.a = 0.5
            marker.color.g = 1.0
            marker.type = 1
            marker.header.frame_id = 'velodyne'
            lifetime = Duration()
            lifetime.nanosec = 300000000
            marker.lifetime = lifetime
            cluster_mask = labels1 == i
            cluster = new_cloud[cluster_mask, :3]

            max_min_diff = np.sqrt(np.sum((np.max(cluster, axis=0)[:2] - np.min(cluster, axis=0)[:2])**2))
            # if cluster is very large...
            if max_min_diff > 1.5:
                # filter points from L shaped clusters by only taking the top half to try to reduce the footprint
                # this is to solve cases where ground points are mixed in with penguin points
                for trial in range(10):
                    # if trial !=0:
                        #print(trial)
                    sorted_by_height = cluster[np.argsort(cluster[:, 2])]
                    top_bit = sorted_by_height[int(9 * len(sorted_by_height) / 10):]
                    top_half = sorted_by_height[int(5 * len(sorted_by_height) / 10):]
                    bottom_half = sorted_by_height[:int(5 * len(sorted_by_height) / 10)]
                    bottom_bit = sorted_by_height[int(1 * len(sorted_by_height) / 10):]

                    top_bit_diff = 2 * np.sqrt(np.sum((np.max(top_bit, axis=0)[:2] - np.min(top_bit, axis=0)[:2])**2))
                    top_diff = 2 * np.sqrt(np.sum((np.max(top_half, axis=0)[:2] - np.min(top_half, axis=0)[:2])**2))
                    bottom_diff = np.sqrt(np.sum((np.max(bottom_half, axis=0)[:2] - np.min(bottom_half, axis=0)[:2])**2))
                    bottom_bit_diff = np.sqrt(np.sum((np.max(bottom_bit, axis=0)[:2] - np.min(bottom_bit, axis=0)[:2])**2))
                    
                    if top_bit_diff < bottom_bit_diff:
                        # print('slicing')
                        cluster = top_half
                    else:
                        break
                    if np.sqrt(np.sum((np.max(cluster, axis=0)[:2] - np.min(cluster, axis=0)[:2])**2)) < 1.5:
                        break
            # for reasonable sized clusters, or clusters with high points...               
            if max_min_diff < 1.5 or np.max(cluster[:, 2] > -0.5):
                # for clusters taller than 20cm
                if np.max(cluster[:, 2]) - np.min(cluster[:, 2]) > 0.2:
                    xs.extend(cluster[:, 0])
                    ys.extend(cluster[:, 1])
                    zs.extend(cluster[:, 2])
                    labels.extend([i] * len(cluster))

                    xmin = np.min(cluster[:, 0])
                    xmax = np.max(cluster[:, 0])
                    # xmean = np.mean(cluster[:, 0], axis=0)
                    
                    ymin = np.min(cluster[:, 1])
                    ymax = np.max(cluster[:, 1])
                    # ymean = np.mean(cluster[:, 1], axis=0)
                    
                    zmin = np.min(cluster[:, 2])
                    zmax = np.max(cluster[:, 2])
                    # zmean = np.mean(cluster[:, 2], axis=0)
                    
                    # Iterate throught the existing markers and check for overlap. If two boxes overlap then we consider them the same objectand taking a bounding bow which fits over both
                    for old_marker in markers.markers:
                        x_old_min = old_marker.pose.position.x - old_marker.scale.x/2
                        x_old_max = old_marker.pose.position.x + old_marker.scale.x/2
                        y_old_min = old_marker.pose.position.y - old_marker.scale.y/2
                        y_old_max = old_marker.pose.position.y + old_marker.scale.y/2

                        if xmin<x_old_min<xmax or xmin<x_old_max<xmax:
                            if ymin<y_old_min<ymax or ymin<y_old_max<ymax:

                                xmin = np.min([xmin,x_old_min])
                                xmax = np.max([xmax,x_old_max])
                                ymin = np.min([ymin,y_old_min])
                                ymax = np.max([ymax,y_old_max])
                                markers.markers.remove(old_marker)

                    xmid = (xmin + xmax)/2
                    ymid = (ymin + ymax)/2
                    zmid = (zmin + zmax)/2                            

                    marker.pose.position.x = xmid
                    marker.pose.position.y = ymid
                    marker.pose.position.z = zmid
                    
                    marker.scale.x = xmax-xmin
                    marker.scale.y = ymax-ymin
                    marker.scale.z = zmax-zmin
                    
                    markers.markers.append(marker)
                    id += 1
            else: # this is to check for penguins on their bellies
                non_cluster_points = new_cloud[new_cloud[:, 3] != i, :3]
                center = np.mean(cluster[:, :2], axis=0)
                # take the 50 closeset points which are not part of the cluster - hopefully these are mostly from the ground
                closest_points = non_cluster_points[np.argsort(
                    (non_cluster_points[:, 0] - center[0])**2 + (non_cluster_points[:, 1] - center[1])**2
                )][:50]

                x_close, y_close, z_close = closest_points[:, :3].T
                local_ground_height = np.mean(z_close)
                # if the height of the current cluster is 15cm higher than the mean of the surrounding points - its a local maximum and we can assume its a penguin
                if np.mean(cluster[:, 2]) > local_ground_height + 0.15:
                    xs.extend(cluster[:, 0])
                    ys.extend(cluster[:, 1])
                    zs.extend(cluster[:, 2])
                    labels.extend([i] * len(cluster))

                    xmin = np.min(cluster[:, 0])
                    xmax = np.max(cluster[:, 0])
                    # xmean = np.mean(cluster[:, 0], axis=0)
                    
                    ymin = np.min(cluster[:, 1])
                    ymax = np.max(cluster[:, 1])
                    # ymean = np.mean(cluster[:, 1], axis=0)
                    
                    zmin = np.min(cluster[:, 2])
                    zmax = np.max(cluster[:, 2])
                    # zmean = np.mean(cluster[:, 2], axis=0)

                    # check overalpping conditions
                    for old_marker in markers.markers:
                        x_old_min = old_marker.pose.position.x - old_marker.scale.x/2
                        x_old_max = old_marker.pose.position.x + old_marker.scale.x/2
                        y_old_min = old_marker.pose.position.y - old_marker.scale.y/2
                        y_old_max = old_marker.pose.position.y + old_marker.scale.y/2

                        if xmin<x_old_min<xmax or xmin<x_old_max<xmax:
                            if ymin<y_old_min<ymax or ymin<y_old_max<ymax:
                                
                                xmin = np.min([xmin,x_old_min])
                                xmax = np.max([xmax,x_old_max])
                                ymin = np.min([ymin,y_old_min])
                                ymax = np.max([ymax,y_old_max])
                                markers.markers.remove(old_marker)

                    xmid = (xmin + xmax)/2
                    ymid = (ymin + ymax)/2
                    zmid = (zmin + zmax)/2                            

                    marker.pose.position.x = xmid
                    marker.pose.position.y = ymid
                    marker.pose.position.z = zmid
                    
                    marker.scale.x = xmax-xmin
                    marker.scale.y = ymax-ymin
                    marker.scale.z = zmax-zmin
    
                    markers.markers.append(marker)
                    id += 1

        header = Header()
        header.frame_id = "velodyne"
        points = np.array([xs,ys,zs]).reshape(3,-1).T
        pc2 = point_cloud2.create_cloud_xyz32(header, points)
        
        self.filtered_cloud_publisher_.publish(pc2) 
        self.marker_publisher_.publish(markers)

def main(args=None):
    rclpy.init(args=args)
    node = LidarDetectionNode()
    rclpy.spin(node)
    rclpy.shutdown()
    

if __name__ == "__main__":
	main()