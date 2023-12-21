import scipy
import numpy as np
import csv
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import  Axes3D
from sklearn.cluster import DBSCAN 
import collections
import time

radius = 8

def get_point_cloud():
    cloud = []
    with open('12.csv','r') as csvfile:
        reader = csv.reader(csvfile)
        for row in reader:
            if row[0] != 'nan':
                row[0] = float(row[0])
                row[1] = float(row[1])
                row[2] = float(row[2])
                cloud.append([float(row[0]), float(row[1]), float(row[2])])
    return cloud
    
def pClosest(points, center):
    K=50
    points.sort(key = lambda K: (K[0]-center[0])**2 + (K[1]-center[1])**2)
    return points[:K]


def cluster_optimized(cloud):
    start = time.time()

    cloud = np.array(cloud)

    mask = np.sqrt(cloud[:, 0]**2 + cloud[:, 1]**2) < radius
    cropped_cloud = cloud[mask]

    xs0, ys0, zs0 = cropped_cloud[:, 0], cropped_cloud[:, 1], cropped_cloud[:, 2]

    xy_points = cropped_cloud[:, :2]

    model = DBSCAN(eps=0.1, min_samples=10)
    model_labels = model.fit_predict(xy_points)

    mask = model_labels != -1
    new_cloud = np.column_stack((xs0[mask], ys0[mask], zs0[mask], model_labels[mask]))

    xs1, ys1, zs1, labels1 = new_cloud[:, 0], new_cloud[:, 1], new_cloud[:, 2], new_cloud[:, 3]

    xs, ys, zs, labels = [], [], [], []

    for i in set(labels1):
        cluster_mask = labels1 == i
        cluster = new_cloud[cluster_mask, :3]

        max_min_diff = np.sqrt(np.sum((np.max(cluster, axis=0)[:2] - np.min(cluster, axis=0)[:2])**2))

        if max_min_diff > 1.5:
            for trial in range(3):
                sorted_by_height = cluster[np.argsort(cluster[:, 2])]
                top_half = sorted_by_height[int(6 * len(sorted_by_height) / 10):]
                bottom_half = sorted_by_height[:int(4 * len(sorted_by_height) / 10)]

                top_diff = 2 * np.sqrt(np.sum((np.max(top_half, axis=0)[:2] - np.min(top_half, axis=0)[:2])**2))
                bottom_diff = np.sqrt(np.sum((np.max(bottom_half, axis=0)[:2] - np.min(bottom_half, axis=0)[:2])**2))

                if top_diff < bottom_diff:
                    cluster = top_half
                else:
                    break
                if max_min_diff < 1.5:
                    break

        if max_min_diff < 1.5:
            if np.max(cluster[:, 2]) - np.min(cluster[:, 2]) > 0.2:
                xs.extend(cluster[:, 0])
                ys.extend(cluster[:, 1])
                zs.extend(cluster[:, 2])
                labels.extend([i] * len(cluster))
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
    print(time.time() - start)
    clusters = [point for point in zip(xs, ys, zs, labels)]
    visualize(cropped_cloud, new_cloud, clusters )
    

def cluster(cloud):
    start = time.time()
    cropped_cloud = []
    for point in cloud:
        if np.sqrt(point[0]*point[0] + point[1]*point[1]) < radius:
            cropped_cloud.append(point)

    xs0, ys0, zs0 = zip(*cropped_cloud)

    model = DBSCAN(eps=.1, min_samples=10)    
    model.fit_predict([point[:2] for point in cropped_cloud])
    new_cloud = [point for point in zip(xs0, ys0, zs0, model.labels_) if point[3] != -1]

    xs1, ys1, zs1, labels1 = zip(*new_cloud)

    # labels1 = set(model.labels_)
    # labels1.remove(-1)
    #print(labels1)

    xs = []
    ys = []
    zs = []
    labels = []

    for i in set(labels1):
    
        cluster = [point[:3] for point in new_cloud if point[3] == i]
        x_clust, y_clust, z_clust = zip(*cluster)

        if np.sqrt((max(x_clust) - min(x_clust))**2 + (max(y_clust) - min(y_clust))**2) > 1.5:
            for trial in range(3):
                #print(trial)
                sorted_by_height = sorted(cluster, key=lambda z:z[2])
                top_half = sorted_by_height[int(6*len(sorted_by_height)/10):]
                bottom_half = sorted_by_height[:int(4*len(sorted_by_height)/10)]
                x_clust_top, y_clust_top, z_clust_top = zip(*top_half)
                x_clust_bottom, y_clust_bottom, z_clust_bottom = zip(*bottom_half)
                #print('top: ', np.sqrt((max(x_clust_top) - min(x_clust_top))**2 + (max(y_clust_top) - min(y_clust_top))**2), 'bottom: ', np.sqrt((max(x_clust_bottom) - min(x_clust_bottom))**2 + (max(y_clust_bottom) - min(y_clust_bottom))**2))
                if 2*np.sqrt((max(x_clust_top) - min(x_clust_top))**2 + (max(y_clust_top) - min(y_clust_top))**2) < np.sqrt((max(x_clust_bottom) - min(x_clust_bottom))**2 + (max(y_clust_bottom) - min(y_clust_bottom))**2):        
                    #print(cluster)
                    cluster = [point for point in zip(x_clust_top, y_clust_top, z_clust_top)]
                    #print(cluster)

                else:
                    break
                if np.sqrt((max(x_clust) - min(x_clust))**2 + (max(y_clust) - min(y_clust))**2) < 1.5:
                    break
        if np.sqrt((max(x_clust) - min(x_clust))**2 + (max(y_clust) - min(y_clust))**2) < 1.5:
            if max(z_clust) - min(z_clust) > 0.2:
                xs_new, ys_new, zs_new = zip(*cluster)
                xs.extend(xs_new)
                ys.extend(ys_new)
                zs.extend(zs_new)
                #print([i]*len(xs_new))
                labels.extend([i]*len(xs_new))
            else:
                non_cluster_points = [point for point in zip(xs0, ys0, zs0, model.labels_) if point[3] != i]
                #closest_points = pClosest(non_cluster_points, )
                #print(non_cluster_points)
                center = [sum(x_clust)/len(x_clust),sum(y_clust)/len(y_clust)]
                K=50
                non_cluster_points.sort(key = lambda K: (K[0]-center[0])**2 + (K[1]-center[1])**2)
                
                closest_points = non_cluster_points[:K]
                

                x_close, y_close, z_close, label_close = zip(*closest_points)
                local_ground_height = sum(z_close)/len(z_close)
                
                if sum(z_clust)/len(z_clust) > local_ground_height + .15:
                    # print('low rider')
                    # print(local_ground_height, sum(z_clust)/len(z_clust))
                    xs_new, ys_new, zs_new = zip(*cluster)
                    xs.extend(xs_new)
                    ys.extend(ys_new)
                    zs.extend(zs_new)               
                    labels.extend([i]*len(xs_new))   
    print(time.time() - start)
    clusters = [point for point in zip(xs, ys, zs, labels)]
    visualize(cropped_cloud, new_cloud, clusters )

def visualize(cloud1, cloud2, cloud3):
    xs0, ys0, zs0 = zip(*cloud1)
    xs1, ys1, zs1, labels1 = zip(*cloud2)
    xs, ys, zs, labels = zip(*cloud3)
    
    fig = plt.figure()
    ax = Axes3D(fig)
    ax.scatter(xs0, ys0, zs0, c='grey', s=1, alpha=0.05)
    # ax.scatter(xs1, ys1, zs1, c=labels1, cmap='prism',s=1, alpha=0.5)
    #ax.scatter(xs1, ys1, zs1, c='black',s=1, alpha=0.25)
    ax.scatter(xs, ys, zs, c=labels, cmap="prism", s=2)
    ax.view_init(azim=200)
    ax.set_xlabel('X Data') # X axis data label
    ax.set_ylabel('Y Data') # Y axis data label
    ax.set_zlabel('Z Data') # Z axis data label
    ax.set_xlim(-radius, radius)
    ax.set_ylim(-radius, radius)
    ax.set_zlim(-radius, radius)
    plt.show()
    # print("number of cluster found: {}".format(len(set(model.labels_))))
    # print("number of cluster after filtering by size: {}".format(len(set(labels))))
    # print('cluster for each point: ', model.labels_)

def main(args=None):
    cloud = get_point_cloud()
    print(cloud[0])
    
    cluster(cloud)
    cluster_optimized(cloud)
    
if __name__ == "__main__":
	main()
