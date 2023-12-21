#! /usr/bin/env python3
# Convert pointcloud obstacle readout to list of penguins

from xml.etree.ElementTree import tostring
import rclpy
import point_cloud2
#import rospy
import scipy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import PoseStamped
import numpy as np
import csv
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
from mpl_toolkits.mplot3d import  Axes3D
from skspatial.objects import Plane, Points
from skspatial.plotting import plot_3d
import matplotlib
from matplotlib import cm
from sklearn.cluster import DBSCAN 
import collections
import time
def flatModel(data, a, b, c, d, e, f, g, h, i, Offset):
    x = data[0]
    y = data[1]
    return a*x + b*y + c*x*x + d*y*y + e*x*y + f*x*x*x + g*y*y*y + h*x*x*y + i*x*y*y + Offset

func = flatModel
initialParameters = [1.0, 1.0, 1.0, 1.0, 1.0]

def get_point_cloud():
    cloud = []
    with open('2.csv','r') as csvfile:
        reader = csv.reader(csvfile)
        for row in reader:
            if row[0] != 'nan':
                row[0] = float(row[0])
                row[1] = float(row[1])
                row[2] = float(row[2])
                cloud.append([float(row[0]), float(row[1]), float(row[2])])
    return cloud

def hypotenuse(x,y):
    hypotenuse = np.sqrt(x*x + y*y)
    return hypotenuse

def SurfacePlot(func, data, fittedParameters):
    graphWidth = 800 # units are pixels
    graphHeight = 600 # units are pixels
    f = plt.figure(figsize=(graphWidth/100.0, graphHeight/100.0), dpi=100)

    matplotlib.pyplot.grid(True)
    axes = Axes3D(f)

    x_data = data[0]
    y_data = data[1]
    z_data = data[2]

    xModel = np.linspace(min(x_data), max(x_data), 20)
    yModel = np.linspace(min(y_data), max(y_data), 20)
    X, Y = np.meshgrid(xModel, yModel)

    Z = func(np.array([X, Y]), *fittedParameters)

    axes.plot_surface(X, Y, Z, rstride=1, cstride=1, cmap=cm.coolwarm, linewidth=1, antialiased=True)

    axes.scatter(x_data, y_data, z_data, marker='.', s=1) # show data along with plotted surface

    axes.set_title('Surface Plot (click-drag with mouse)') # add a title for surface plot
    axes.set_xlabel('X Data') # X axis data label
    axes.set_ylabel('Y Data') # Y axis data label
    axes.set_zlabel('Z Data') # Z axis data label

    plt.show()
    plt.close('all') 

def crop(cloud):
    new_cloud = []
    for point in cloud:
        if hypotenuse(point[0], point[1]) < 10:
            new_cloud.append(point)
    return new_cloud


def lidar_detect(cloud):
    xs = []
    ys = []
    zs = []
    for point in cloud:
        xs.append(point[0])
        ys.append(point[1])
        zs.append(point[2])
    
    fittedParameters, pcov = scipy.optimize.curve_fit(func, [xs, ys], zs)
    print('Fitted Parameters: ', fittedParameters)
    
    #cloud = crop(cloud)
    print(len(cloud))
    
    obstacle_cloud = []
    for point in cloud:
        z_fit = func(np.array([point[0], point[1]]), *fittedParameters)
        #if point[2] - z_fit > .1:
        obstacle_cloud.append(point)
    
    print(len(obstacle_cloud))
    xs = []
    ys = []
    zs = []
    for point in obstacle_cloud:
        xs.append(point[0])
        ys.append(point[1])
        zs.append(point[2])    
    #SurfacePlot(func, [xs, ys, zs], fittedParameters)    
    return obstacle_cloud

def cluster(cloud):
    xs0 = []
    ys0 = []
    zs0 = []

    for i in range(len(cloud)):
        point = cloud[i]
        xs0.append(point[0])
        ys0.append(point[1])
        zs0.append(point[2])
            
    model = DBSCAN(eps=.08, min_samples=10)
    xy = []
    for point in cloud:
        xy.append([point[0], point[1]])

    model.fit_predict(xy)
    print(model.labels_)

    xs1 = []
    ys1 = []
    zs1 = []
    labels1 = []

    new_cloud = []
    for i in range(len(cloud)):
        point = cloud[i]
        if model.labels_[i] != -1:
            xs1.append(point[0])
            ys1.append(point[1])
            zs1.append(point[2])
            labels1.append(model.labels_[i])
            new_cloud.append([point[0],point[1],point[2],model.labels_[i]])

    d = collections.defaultdict(list)
    for sub in new_cloud:
        d[sub[3]].append(sub)

    xs = []
    ys = []
    zs = []
    labels = []
    for i in range(len(d)):
        cluster = d[i]
        x_min = min(cluster, key=lambda x: x[0])
        y_min = min(cluster, key=lambda y: y[1])
        z_min = min(cluster, key=lambda z: z[2])
        x_max = max(cluster, key=lambda x: x[0])
        y_max = max(cluster, key=lambda y: y[1])
        z_max = max(cluster, key=lambda z: z[2])
        if z_max[2] - z_min[2] > 0.2:
            if x_max[0] - x_min[0] < 3.0:
                if y_max[1] - y_min[1] < 3.0:
                    for point in cluster:
                        xs.append(point[0])
                        ys.append(point[1])
                        zs.append(point[2])
                        labels.append(point[3])     

    fig = plt.figure()
    ax = Axes3D(fig)
    ax.scatter(xs0, ys0, zs0, c='black', s=1, alpha=0.05)
    ax.scatter(xs1, ys1, zs1, c='grey', s=1, alpha=0.25)
    ax.scatter(xs, ys, zs, c=labels, cmap="prism", s=1)
    ax.view_init(azim=200)
    ax.set_xlabel('X Data') # X axis data label
    ax.set_ylabel('Y Data') # Y axis data label
    ax.set_zlabel('Z Data') # Z axis data label
    ax.set_xlim(-10, 10)
    ax.set_ylim(-10, 10)
    ax.set_zlim(-10, 10)
    plt.show()
    print("number of cluster found: {}".format(len(set(model.labels_))))
    print("number of cluster after filtering by size: {}".format(len(set(labels))))
    print('cluster for each point: ', model.labels_)

def main(args=None):
    cloud = get_point_cloud()
    start = time.time()
    filtered_cloud = lidar_detect(cloud)
    clusters = cluster(filtered_cloud)
    print(time.time() - start)

if __name__ == "__main__":
	main()
