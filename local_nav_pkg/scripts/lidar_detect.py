#! /usr/bin/env python3

# Convert pointcloud obstacle readout to list of penguins

from xml.etree.ElementTree import tostring
import rclpy
import point_cloud2
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import MarkerArray, Marker
from std_msgs.msg import Header
from builtin_interfaces.msg import Duration
import numpy as np
from sklearn.cluster import DBSCAN 

cloud = [[0.0, 0.0, 0.0]]
radius = 12

SIMULATION = False

class LidarDetectionNode(Node):

    def __init__(self):
        super().__init__('lidar_object_publisher')
        self.subscriber = self.create_subscription(PointCloud2, 'velodyne_points', self.velodyne_callback, 1)
        
        self.filtered_cloud_publisher_ = self.create_publisher(PointCloud2, 'filtered_cloud', 1)
        self.marker_publisher_ = self.create_publisher(MarkerArray, 'visualization_marker_array', 1)
        self.cloud_timer_ = self.create_timer(.1, self.cluster)
        
    def velodyne_callback(self, msg):
        global cloud
        try:
            cloud_raw = point_cloud2.read_points(msg)
            cloud = []
            for point in cloud_raw:
                cloud.append([point[0],point[1],point[2]])
            
        except IndexError:
            print('no cloud detected yet')
                       
    def cluster(self):
        global cloud
        cloud = np.array(cloud)
        #print(cloud)
        mask = np.sqrt(cloud[:, 0]**2 + cloud[:, 1]**2) < radius
        cropped_cloud = cloud[mask]
        if SIMULATION:
            mask2 = cropped_cloud[:, 2] > -.5
            cropped_cloud = cropped_cloud[mask2]

        xs0, ys0, zs0 = cropped_cloud[:, 0], cropped_cloud[:, 1], cropped_cloud[:, 2]

        xy_points = cropped_cloud[:, :2]

        model = DBSCAN(eps=0.075, min_samples=10)
        model_labels = model.fit_predict(xy_points)

        mask = model_labels != -1
        new_cloud = np.column_stack((xs0[mask], ys0[mask], zs0[mask], model_labels[mask]))

        xs1, ys1, zs1, labels1 = new_cloud[:, 0], new_cloud[:, 1], new_cloud[:, 2], new_cloud[:, 3]

        xs, ys, zs, labels = [], [], [], []
        markers = MarkerArray()
        id = 0
        for i in set(labels1):
            marker = Marker()
            marker.id = id
            marker.ns = 'cluster'
            marker.color.a = 0.5
            marker.color.g = 1.0
            marker.type = 1
            marker.header.frame_id = 'velodyne'
            lifetime = Duration()
            lifetime.nanosec = 200000000
            marker.lifetime = lifetime
            cluster_mask = labels1 == i
            cluster = new_cloud[cluster_mask, :3]

            max_min_diff = np.sqrt(np.sum((np.max(cluster, axis=0)[:2] - np.min(cluster, axis=0)[:2])**2))

            if max_min_diff > 1.5:
                
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

            if max_min_diff < 1.5 or np.max(cluster[:, 2] > -0.5):
                if np.max(cluster[:, 2]) - np.min(cluster[:, 2]) > 0.2:
                    xs.extend(cluster[:, 0])
                    ys.extend(cluster[:, 1])
                    zs.extend(cluster[:, 2])
                    labels.extend([i] * len(cluster))

                    marker.pose.position.x = np.mean(cluster[:, 0], axis=0)
                    marker.pose.position.y = np.mean(cluster[:, 1], axis=0)
                    marker.pose.position.z = np.mean(cluster[:, 2], axis=0)

                    marker.scale.x = np.max(cluster[:, 0]) - np.min(cluster[:, 0])
                    marker.scale.y = np.max(cluster[:, 1]) - np.min(cluster[:, 1])
                    marker.scale.z = np.max(cluster[:, 2]) - np.min(cluster[:, 2])
                    
                    markers.markers.append(marker)
                    id += 1
                else:
                    non_cluster_points = new_cloud[new_cloud[:, 3] != i, :3]
                    center = np.mean(cluster[:, :2], axis=0)
                    closest_points = non_cluster_points[np.argsort(
                        (non_cluster_points[:, 0] - center[0])**2 + (non_cluster_points[:, 1] - center[1])**2
                    )][:50]

                    x_close, y_close, z_close = closest_points[:, :3].T
                    local_ground_height = np.mean(z_close)

                    if np.mean(cluster[:, 2]) > local_ground_height + 0.15:
                        xs.extend(cluster[:, 0])
                        ys.extend(cluster[:, 1])
                        zs.extend(cluster[:, 2])
                        labels.extend([i] * len(cluster))

                        marker.pose.position.x = np.mean(cluster[:, 0], axis=0)
                        marker.pose.position.y = np.mean(cluster[:, 1], axis=0)
                        marker.pose.position.z = np.mean(cluster[:, 2], axis=0)

                        marker.scale.x = np.max(cluster[:, 0]) - np.min(cluster[:, 0])
                        marker.scale.y = np.max(cluster[:, 1]) - np.min(cluster[:, 1])
                        marker.scale.z = np.max(cluster[:, 2]) - np.min(cluster[:, 2])
        
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