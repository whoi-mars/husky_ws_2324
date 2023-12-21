//#pragma once
#include <memory>
#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
// #include <boost/foreach.hpp>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>

// Include files required for RANSAC
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>

#include <pcl/filters/statistical_outlier_removal.h>

#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h> //Used for getMinMax3D
#include <pcl/common/centroid.h> //Used for cluster centroid calculation
#include <fstream>
#include <sstream>
#include <string>
#include <math.h>
#include <vector>

// Include files for socket
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <string.h>
#include <stdio.h>

//To see bounding box in rviz2
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>


#define SERVER_PORT htons(65432)
char socket_buffer[1000];
int serverSock;
int clientSock;
socklen_t sin_size=sizeof(struct sockaddr_in);
sockaddr_in clientAddr;
visualization_msgs::msg::Marker fov_marker_pts; //Ptr usage is deprecated warning: ‘visualization_msgs::msg::Marker_<std::allocator<void> >::Ptr’ is deprecated [-Wdeprecated-declarations]   96 |   visualization_msgs::msg::Marker::Ptr fov_marker_pts;

// Need to stop the robot if the obstacle is too close

// using std::placeholders::_1;

// clear && colcon build --packages-select custom_nav_stack_pkg --symlink-install && source install/local_setup.bash && ros2 run custom_nav_stack_pkg cpp_executable
// sudo chown -R dev /opt/ros/foxy/share/ #to remove the squiggle error in vscode where "dev" is the username
// sudo apt install pcl-tools to install pcl_viewer to visualize the cluster for debugging

// LIMITATION: 1) If object is too close to the LiDAR, it won't come in the pointcloud or might come partial only

class MinimalSubscriber : public rclcpp::Node
{

  struct Config {
        int    num;
        std::string str;
        double flt;
        float leaf_size_x;
        float leaf_size_y;
        float leaf_size_z;
        float crop_min_x;
        float crop_min_y;
        float crop_min_z;
        float crop_max_x;
        float crop_max_y;
        float crop_max_z;
        int ransac_max_iterations;
        float ransac_distance_threshold;
        float cluster_tolerance;
        int cluster_min_size;
        int cluster_max_size;
        float fwd_obs_tolerance;
        float angle_view_degree;
      };
  Config config;
  float angle_view_radian;
  
  
  public:
    
    int detect_obstacle(float xmin, float ymin, float xmax, float ymax, float *angle_individual_radian_ptr) const
    {
      std::cout << "Angle view radian, xmin, ymin, xmax, ymax, rctheta, rstheta " << angle_view_radian << ", " << xmin << ", " << ymin << ", "  << xmax << ", "  << ymax << ", "  << config.fwd_obs_tolerance*cos(angle_view_radian) << ", "  << config.fwd_obs_tolerance*sin(angle_view_radian) << ", "  <<   std::endl;
      
      // Conditions to detect cluster/ obstacle AABB (Axis-Aligned Bounding Box) rectangle defined by xmin, ymin, xmax, ymax within a circle's sector defined by config.fwd_obs_tolerance and angle_view_degree
      if (xmin <= 0 && xmax <= 0){
        std::cout << 1 << std::endl; return 0;} // Case 0: If all x values are -ve, then the bounding box/ rectangle/ cluster/ obstacle is behind the robot and its a don't care condition.
      else if (ymin >= config.fwd_obs_tolerance*sin(angle_view_radian)){std::cout << 2 << std::endl; return 0;} // Case 1: Rectangle lies to the left of robot
      else if (ymax <= -config.fwd_obs_tolerance*sin(angle_view_radian)){std::cout << 3 << std::endl; return 0;} // Case 2: Rectangle lies to the right of robot. Case 3 = 1 and 4 = 2
      // Rectangle is not at the back, left, right of robot. It can be partial overlapping or no overlapping. Checking for further possibilities.
      else if (ymin >= 0 && ymin <= config.fwd_obs_tolerance*sin(angle_view_radian)){
        std::cout << 4 << std::endl;
        if (xmin >= config.fwd_obs_tolerance){std::cout << 5 << std::endl; return 0;} // Case 5: xmin >= r
        else if (xmin >= config.fwd_obs_tolerance*cos(angle_view_radian)){ // xmin < r naturally because of last if
          std::cout << 6 << std::endl;
          if (sqrt(xmin*xmin + ymin*ymin) >= config.fwd_obs_tolerance){std::cout << 7 << std::endl; return 0;}  // Case 7 : check distance of BR, if >= r, outside POV
          else {std::cout << 8 << std::endl; *angle_individual_radian_ptr = atan2(ymin, xmin); return 1;} // Case 7 : check distance of BR, if < r, inside POV and obstacle is detected
        }
        else if (xmax > config.fwd_obs_tolerance*cos(angle_view_radian)){std::cout << 9 << std::endl; *angle_individual_radian_ptr = atan2(ymin, xmin); return 1;} // Case 11: xmax > x_L; xmin < r and xmin < x_L naturally because of last else if. Right side of rectangle crosses the sector, hence obstacle detected
        else if ((ymin - (tan(angle_view_radian))*xmax)*(-(tan(angle_view_radian))*config.fwd_obs_tolerance) > 0){std::cout << 10 << std::endl; *angle_individual_radian_ptr = atan2(ymin, xmax); return 1;}// Case 9: Need to check if TR (xmax, ymax) and (r,0) are on same side of y = (y_L/x_L)x line. If same side, obstacle detected, otherwise not.
        else {std::cout << 11 << std::endl; return 0;} // TR is on opposite side of (r,0), hence inside of common area and obstacle is detected.
      }
      else if (ymax <= 0 && ymax >= -config.fwd_obs_tolerance*sin(angle_view_radian)){ // Case 6
        std::cout << 12 << std::endl;
        if (xmin >= config.fwd_obs_tolerance){std::cout << 13 << std::endl; return 0;} // Case 6: xmin >= r
        else if (xmin >= config.fwd_obs_tolerance*cos(angle_view_radian)){ // xmin < r naturally because of last if. If xmin >= x_R or r*cos(theta), check distance of BL from (0,0)
          std::cout << 14 << std::endl;
          if(sqrt(xmin*xmin + ymax*ymax) >= config.fwd_obs_tolerance){std::cout << 15 << std::endl; return 0;} // Case 8: If BL distance >=r obstacle not detected
          else {std::cout << 16 << std::endl; *angle_individual_radian_ptr = atan2(ymax, xmin); return 1;} // Case 8: If BL distance < r, obstacle detected
        }
        else if (xmax > config.fwd_obs_tolerance*cos(angle_view_radian)){std::cout << 18 << std::endl; *angle_individual_radian_ptr = atan2(ymin, xmin); return 1;} // Case 12: xmax > x_R and xmin< x_R naturally; Left side of rectangle crosses the sector, hence obstacle detected
        else if ((ymax + tan(angle_view_radian)*xmax)*(tan(angle_view_radian)*config.fwd_obs_tolerance) > 0){
          std::cout << 19 << std::endl; *angle_individual_radian_ptr = atan2(ymax, xmax); return 1;} // Case 10: xmax < x_R naturally; TL and (r,0) are on the same side of FOV right line, hence obstacle is detected otherwise not
        else {std::cout << 20 << std::endl; return 0;}
      }
      else if (ymax >= 0 && ymin <= 0){
        std::cout << 21 << std::endl;
        if (xmin >= config.fwd_obs_tolerance){std::cout << 22 << std::endl; return 0;}
        else {
          std::cout << 23 << std::endl;
          if (abs(ymax) >= abs(ymin)){*angle_individual_radian_ptr = atan2(ymin, xmin);}else{*angle_individual_radian_ptr = atan2(ymax, xmin);}
          return 1;} // Case 13: Rectangle bottom side crosses the sector
      }
      // No if-else condition matched (though it should not happen), so return 0 to avoid the warning while compiling: "/home/dev/husky_ws/src/custom_nav_stack_pkg/src/cpp_node.cpp: In member function ‘int MinimalSubscriber::detect_obstacle(float, float, float, float) const’:/home/dev/husky_ws/src/custom_nav_stack_pkg/src/cpp_node.cpp:126:5: warning: control reaches end of non-void function [-Wreturn-type]  126 |     }" 
      return 0;
    }

    MinimalSubscriber()    //Constructor which has the same name as that of class followed by a parentheses. A constructor in C++ is a special method that is automatically called when an object of a class is created. 
    : Node("minimal_subscriber")
    {
      std::cout << "Reached start of Public" << std::endl;
      
      
      std::ifstream config_file_path("/home/administrator/husky_ws2/src/custom_nav_stack_pkg/src/config2.txt");
      std::string line;
      while (std::getline(config_file_path, line)) {
          std::istringstream sin(line.substr(line.find("=") + 1));
          if (line.find("num") == 0)
              sin >> config.num;
          else if (line.find("str") == 0)
              sin >> config.str;
          else if (line.find("flt") == 0)
              sin >> config.flt;
          else if (line.find("leaf_size_x") == 0)
              sin >> config.leaf_size_x;
          else if (line.find("leaf_size_y") == 0)
              sin >> config.leaf_size_y;
          else if (line.find("leaf_size_z") == 0)
              sin >> config.leaf_size_z;
          else if (line.find("crop_min_x") == 0)
              sin >> config.crop_min_x;
          else if (line.find("crop_min_y") == 0)
              sin >> config.crop_min_y;
          else if (line.find("crop_min_z") == 0)
              sin >> config.crop_min_z;
          else if (line.find("crop_max_x") == 0)
              sin >> config.crop_max_x;
          else if (line.find("crop_max_y") == 0)
              sin >> config.crop_max_y;
          else if (line.find("crop_max_z") == 0)
              sin >> config.crop_max_z;
          else if (line.find("ransac_max_iterations") == 0)
              sin >> config.ransac_max_iterations;
          else if (line.find("ransac_distance_threshold") == 0)
              sin >> config.ransac_distance_threshold;
          else if (line.find("cluster_tolerance") == 0)
              sin >> config.cluster_tolerance;
          else if (line.find("cluster_min_size") == 0)
              sin >> config.cluster_min_size;
          else if (line.find("cluster_max_size") == 0)
              sin >> config.cluster_max_size;
          else if (line.find("fwd_obs_tolerance") == 0)
              sin >> config.fwd_obs_tolerance;
          else if (line.find("angle_view_degree") == 0)
              sin >> config.angle_view_degree;
      }
      angle_view_radian = config.angle_view_degree * (M_PI / 360); //Dividing it by 360 instead of 180 to get theta/2 instead of theta
      std::cout << config.fwd_obs_tolerance << '\n';
      std::cout << config.angle_view_degree << '\n';
      std::cout << config.crop_min_x << '\n';
      std::cout << config.crop_max_x << '\n';
      
      
      fov_marker_pts.ns = "fov_lidar";
      fov_marker_pts.id = 0;
      fov_marker_pts.type = visualization_msgs::msg::Marker::POINTS;
      fov_marker_pts.action = visualization_msgs::msg::Marker::ADD;
      fov_marker_pts.pose.orientation.x = 0.0;
      fov_marker_pts.pose.orientation.y = 0.0;
      fov_marker_pts.pose.orientation.z = 0.0;
      fov_marker_pts.pose.orientation.w = 1.0;
      fov_marker_pts.scale.x = 0.1;
      fov_marker_pts.scale.y = 0.1;
      fov_marker_pts.color.r = 1.0f;
      fov_marker_pts.color.g = 0.647f;
      fov_marker_pts.color.b = 0.0f;
      fov_marker_pts.color.a = 0.2;

      geometry_msgs::msg::Point p;
      // Create the vertices for the points and lines (Ref: http://wiki.ros.org/rviz/Tutorials/Markers%3A%20Points%20and%20Lines)
      for (uint32_t i = 0; i < 50; ++i){
        // Generating x and y points for left and right bounding lines
        float x = (config.fwd_obs_tolerance * i * cos(angle_view_radian))/49;
        float y_l = (config.fwd_obs_tolerance * i * sin(angle_view_radian))/49;
        float y_r = -(config.fwd_obs_tolerance * i * sin(angle_view_radian))/49;
        p.x = x;
        p.y = y_l;
        fov_marker_pts.points.push_back(p);
        p.y = y_r;
        fov_marker_pts.points.push_back(p);
        //Generating points for circular arc
        y_l = (i*config.fwd_obs_tolerance*sin(angle_view_radian))/49;
        x = sqrt(config.fwd_obs_tolerance*config.fwd_obs_tolerance - y_l*y_l);
        p.x = x;
        p.y = y_l;
        fov_marker_pts.points.push_back(p);
        p.y = -y_l;
        fov_marker_pts.points.push_back(p);
        //
        p.y = 0;
        p.x = i*config.fwd_obs_tolerance/49;
        fov_marker_pts.points.push_back(p);
      }

      
      
      publisher_pcl = this->create_publisher<sensor_msgs::msg::PointCloud2>("filtered_cloud", 10);
      publisher_marker_array = this->create_publisher<visualization_msgs::msg::MarkerArray>("visualization_marker_array", 10);
      
      subscription_vlp16 = this->create_subscription<sensor_msgs::msg::PointCloud2>
      ("velodyne_points", rclcpp::SensorDataQoS(), std::bind(&MinimalSubscriber::topic_callback, this, std::placeholders::_1));

      std::cout << "Reached end of Public" << std::endl;      
    }

  private:

    
    void topic_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg_ptr) const
    {
      std::cout << "Hello" << std::endl;
      sleep(0.5);
      // RCLCPP_INFO(this->get_logger(), "I received the message , height is: '%d'", msg_ptr->height); //
      // RCLCPP_INFO(this->get_logger(), "I received the message"); //

      // Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud
      // pcl::PointCloud<pcl::PointXYZ> cloud; //Working, Reference: https://ros-developer.com/2017/02/23/converting-pcl-point-cloud-to-ros-pcl-cloud-message-and-the-reverse/
      // pcl::fromROSMsg (*msg, cloud); //Working
      // RCLCPP_INFO(this->get_logger(), "I received the PCL message , height is: '%d'", cloud.height); //WORKING
      // pcl::PCLPointCloud2::Ptr cloud;      
      // pcl::fromROSMsg (*msg, cloud);
      pcl::PCLPointCloud2::Ptr cloud_in_ptr(new pcl::PCLPointCloud2);
      pcl_conversions::toPCL(*msg_ptr, *cloud_in_ptr);
      // pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
      // pcl::fromPCLPointCloud2(*cloud,*temp_cloud);
      // RCLCPP_INFO(this->get_logger(), "I received the PCL message , height is: '%d'", cloud.height);
      // RCLCPP_INFO_STREAM(get_logger(), "[Input PCL PointCloud] width " << cloud_in_ptr->width << " height " << cloud_in_ptr->height);
      // cloud.points[:].x = 4;
      // std::cout << "Complete ROS cloud is: " << msg << std::endl;
      // std::cout << "Complete ROS cloud type is: " << typeid(msg).name() << std::endl;
      // std::cout << "Complete PCL cloud is: " << cloud << std::endl;      
      // std::cout << "Complete PCL cloud type is: " << typeid(cloud).name() << std::endl;
      // std::cout << "PCL cloud points type is: " << typeid(cloud.points).name() << std::endl;
      // std::cout << "PCL 0 cloud is: " << cloud.points[0] << std::endl;      
      // std::cout << "PCL 15000 cloud is: " << cloud.points[15000] << std::endl;      
      // std::cout << "PCL 25000 cloud is: " << cloud.points[25000] << std::endl;      

 
      // // loop through the array elements
      // for (size_t i = 0; i < n; i++) {
      //     std::cout << input[i] << ' ';
      // }
      //Reading xyz values from the point cloud
      // BOOST_FOREACH (const pcl::PointXYZ& pt, cloud.points)
      // printf ("\t(%f, %f, %f)\n", pt.x, pt.y, pt.z);
      

      
      std::cerr << "PointCloud before filtering: " << cloud_in_ptr->width * cloud_in_ptr->height 
             << " data points (" << pcl::getFieldsList (*cloud_in_ptr) << ")." << std::endl;

      
      // Declaring pointer for the PCL "cloud" 
      // pcl::PCLPointCloud2::Ptr cloudPtr (&cloud);

      // define a new container for the data
      pcl::PCLPointCloud2::Ptr cloud_filtered_ptr (new pcl::PCLPointCloud2 ());

      // STEP 1: Filtering Using VoxelGrid; Reference: https://codediversion.wordpress.com/2019/01/16/simple-ros-c-and-lidar-with-pcl-on-ubuntu-16-04/

      // define a voxelgrid
      pcl::VoxelGrid<pcl::PCLPointCloud2> voxelGrid;
      // set input to cloud
      voxelGrid.setInputCloud(cloud_in_ptr);
      // set the leaf size (x, y, z)
      //voxelGrid.setLeafSize(.1, .1, .1);
      voxelGrid.setLeafSize(config.leaf_size_x, config.leaf_size_y, config.leaf_size_z);
      // apply the filter to dereferenced cloudVoxel
      voxelGrid.filter(*cloud_filtered_ptr);   

      std::cerr << "PointCloud after voxel filtering: " << cloud_filtered_ptr->width * cloud_filtered_ptr->height 
       << " data points (" << pcl::getFieldsList (*cloud_filtered_ptr) << ")." << std::endl;

      
      
      // STEP 2: Cropping Using CropBox; Reference: https://answers.ros.org/question/318183/how-to-use-cropbox-on-pointcloud2/

      // define a cropbox
      pcl::CropBox<pcl::PCLPointCloud2> cropBox;
      cropBox.setInputCloud(cloud_filtered_ptr);
      ///Eigen::Vector4f min_pt (-50.0f, -50.0f, -500.0f, 1.0); //(minX, minY, minZ, 1.0) in meter
      //Eigen::Vector4f max_pt (50.0f, 50.0f, 50.0f, 1.0); //(maxX, maxY, maxZ, 1.0) in meter
      // float my_x = -5.1f;
      // std::cout << "Config crop min x is: " << config.crop_min_x << std::endl;
      // std::cout << "My x is: " << my_x << "of type" << typeid(my_x).name() << std::endl;
      Eigen::Vector4f min_pt (config.crop_min_x, config.crop_min_y, config.crop_min_z, 1.0f); //(minX, minY, minZ, 1.0) in meter
      Eigen::Vector4f max_pt (config.crop_max_x, config.crop_max_y, config.crop_max_z, 1.0f);
      // Cropbox slighlty bigger then bounding box of points
      cropBox.setMin (min_pt);
      cropBox.setMax (max_pt);
      // Indices
      std::vector<int> indices;
      cropBox.filter (indices);
      // Cloud
      cropBox.filter(*cloud_filtered_ptr);
      std::cerr << "PointCloud after cropbox filtering: " << cloud_filtered_ptr->width * cloud_filtered_ptr->height 
       << " data points (" << pcl::getFieldsList (*cloud_filtered_ptr) << ")." << std::endl;

      // STEP 3(a): Segmentation Using RANSAC Algorithm; Reference: https://pointclouds.org/documentation/classpcl_1_1_s_a_c_segmentation.html#a7e9ad0f4cd31e45c2ff03da17d0c9bce, https://pcl.readthedocs.io/projects/tutorials/en/latest/planar_segmentation.html, https://github.com/RAS2015-GROUP5/ras_computer_vision/blob/master/milestone_1_object_detection/src/object_detection.cpp
      pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
      pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
      // Create the segmentation object
      // pcl::SACSegmentation<pcl::PCLPointCloud2> seg;
      pcl::SACSegmentation<pcl::PointXYZ> seg;
      
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz_ptr(new pcl::PointCloud<pcl::PointXYZ>);
      pcl::fromPCLPointCloud2(*cloud_filtered_ptr, *cloud_xyz_ptr);  //https://stackoverflow.com/questions/69716482/convert-from-pclpointcloudpclpointxyz-to-pclpclpointcloud2-ros-melodic  (we can using <PointT> or PointXYZ in all function it seems)
      // cloud_xyz_ptr->header =cloud_filtered_ptr->header;
      // cloud_xyz_ptr->width =cloud_filtered_ptr->width;
      // cloud_xyz_ptr->height =cloud_filtered_ptr->height;
      // cloud_xyz_ptr->is_dense =cloud_filtered_ptr->is_dense;
      // cloud_xyz_ptr->points =cloud_filtered_ptr->data;

      // Optional
      seg.setOptimizeCoefficients (true);
      // Mandatory
      seg.setModelType (pcl::SACMODEL_PLANE);
      seg.setMethodType (pcl::SAC_RANSAC);
      //seg.setMaxIterations (500);
      //seg.setDistanceThreshold (0.1);
      
      seg.setMaxIterations (config.ransac_max_iterations);
      seg.setDistanceThreshold (config.ransac_distance_threshold);
      
      seg.setInputCloud (cloud_xyz_ptr);
      seg.segment (*inliers, *coefficients);

      if (inliers->indices.size () == 0)
      {
        PCL_ERROR ("Could not estimate a planar model for the given dataset.");
        std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
        //Need to skip ransac part here
      }

      std::cerr << "Model coefficients: " << coefficients->values[0] << " " 
                                          << coefficients->values[1] << " "
                                          << coefficients->values[2] << " " 
                                          << coefficients->values[3] << std::endl;

      std::cerr << "Model inliers: " << inliers->indices.size () << std::endl;

      
      // STEP 3(b): Extracting ground/ non-ground points // https://github.com/jupidity/PCL-ROS-cluster-Segmentation/blob/master/README.md
        // If we want ground points, use extract.setNegative (false);
        // If we want non-ground, obstacle points, use extract.setNegative (true);
      
      // Create the filtering object
      pcl::ExtractIndices<pcl::PointXYZ> extract;

      extract.setInputCloud (cloud_xyz_ptr);
      extract.setIndices (inliers);
      extract.setNegative (true);
      extract.filter (*cloud_xyz_ptr);

      std::cerr << "PointCloud after RANSAC plane extraction, non-ground points: " << cloud_xyz_ptr->width * cloud_xyz_ptr->height 
       << " data points (" << pcl::getFieldsList (*cloud_xyz_ptr) << ")." << std::endl;

      // Create the filtering object
      pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;

      sor.setInputCloud (cloud_xyz_ptr);
      sor.setMeanK (50);
      sor.setStddevMulThresh (1.0);
      //sor.filter (*cloud_xyz_ptr);

      if (cloud_xyz_ptr->width * cloud_xyz_ptr->height != 0){ //Don't do clustering if non-ground points are not detected.
        // Step 4: Clustering using K-D treel; Reference: https://link.springer.com/chapter/10.1007/978-981-16-6460-1_57, https://github.com/jupidity/PCL-ROS-cluster-Segmentation/blob/master/README.md, https://pcl.readthedocs.io/en/latest/cluster_extraction.html

        // Create the KdTree object for the search method of the extraction
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
        tree->setInputCloud (cloud_xyz_ptr);

        // create the extraction object for the clusters
        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
        // specify euclidean cluster parameters
        //ec.setClusterTolerance (.1); // 2cm
        //ec.setMinClusterSize (30);
        //ec.setMaxClusterSize (25000);
        ec.setClusterTolerance (config.cluster_tolerance); // 2cm
        ec.setMinClusterSize (config.cluster_min_size);
        ec.setMaxClusterSize (config.cluster_max_size);
        ec.setSearchMethod (tree);
        ec.setInputCloud (cloud_xyz_ptr);
        // exctract the indices pertaining to each cluster and store in a vector of pcl::PointIndices
        ec.extract (cluster_indices);
        // std::cout << "Cluster indices: " << cluster_indices[0] << std::endl;

        pcl::PCDWriter writer;

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster_all (new pcl::PointCloud<pcl::PointXYZ>); //Point cloud containig all the clusters. Was implemented to visualise all the clusters simultaneously in RVIZ2 instead of sequentially.
        std::vector <std::vector <float>> min_pt_cluster_all(cluster_indices.size()); //Array of vector of size of no. of clusters containing min points of each cluster x1,y1,z1 ; x2.y2,z2 ...
        std::vector <std::vector <float>> max_pt_cluster_all(cluster_indices.size()); //Array of vector of size of no. of clusters containing max points of each cluster x1,y1,z1 ; x2.y2,z2 ...
        // NOTE: xmin, ymin, zmin are not paired. They can be different points. They are just min values in x, y, and z respectively.
        std::vector <std::vector <float>> centroid_cluster_all(cluster_indices.size()); //Array of vector of size of no. of clusters containing the centroid points co-ordinates x,y,z which are paired.
        std::vector <std::vector <float>> centroid_direction_array(cluster_indices.size()); // Array of angles in radian containing the angle at which centroid of a cluster is present wrt robot
        std::vector <std::vector <float>> angle_corner_radian_all(cluster_indices.size()); // Array of angles of AABB wrt lidar which will help in finding by how much angle to rotate the robot
        visualization_msgs::msg::MarkerArray marker_all;
        
        uint32_t shape = visualization_msgs::msg::Marker::CUBE;

        fov_marker_pts.header = msg_ptr->header;

        int j = 0;
        for (const auto& cluster : cluster_indices)
        {
          std::cout << "Number of clusters: " << cluster_indices.size() << std::endl;
          pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster_individual (new pcl::PointCloud<pcl::PointXYZ>);
          // pcl::CentroidPoint<pcl::PointXYZ> centroid; //Reference: https://pointclouds.org/documentation/classpcl_1_1_centroid_point.html
          for (const auto& idx : cluster.indices) {
          cloud_cluster_all->push_back((*cloud_xyz_ptr)[idx]);
          cloud_cluster_individual->push_back((*cloud_xyz_ptr)[idx]);
          } //*
          cloud_cluster_all->width = cloud_cluster_all->size ();
          cloud_cluster_all->height = 1;
          cloud_cluster_all->is_dense = true;   
          cloud_cluster_all->header = cloud_xyz_ptr->header;  
          std::cout << "PointCloud representing the Cluster# " << j << " is of size: " << cloud_cluster_all->size () << " data points." << std::endl;

          Eigen::Vector4f min_pt_cluster; //(minX, minY, minZ, 1.0) in meter
          Eigen::Vector4f max_pt_cluster; //(maxX, maxY, maxZ, 1.0) in meter
          pcl::getMinMax3D(*cloud_cluster_individual,min_pt_cluster, max_pt_cluster); //Ref: https://github.com/PointCloudLibrary/pcl/blob/master/examples/common/example_get_max_min_coordinates.cpp
          min_pt_cluster_all[j].push_back(min_pt_cluster[0]);
          min_pt_cluster_all[j].push_back(min_pt_cluster[1]);
          min_pt_cluster_all[j].push_back(min_pt_cluster[2]);
          max_pt_cluster_all[j].push_back(max_pt_cluster[0]);
          max_pt_cluster_all[j].push_back(max_pt_cluster[1]);
          max_pt_cluster_all[j].push_back(max_pt_cluster[2]);

          // Finding centroid of the individual cluster and storing them with centroid co-ords of all clusters; Ref: https://pointclouds.org/documentation/classpcl_1_1_centroid_point.html
          Eigen::Vector4d centroid;
          pcl::compute3DCentroid (*cloud_cluster_individual, centroid);
          // pcl::PointXYZ c1;
          // centroid.add(cloud_cluster_individual); //c1 contains centroid xyz co-ords of the jth cluster
          // centroid_cluster_all[j].push_back(c1.x);
          // centroid_cluster_all[j].push_back(c1.y);
          // centroid_cluster_all[j].push_back(c1.z);

          // centroid_direction_array[j].push_back(atan2(centroid_cluster_all[j][1], centroid_cluster_all[j][0]));
          centroid_direction_array[j].push_back(atan2(centroid[1], centroid[0]));
          centroid_cluster_all[j].push_back(centroid[0]);
          centroid_cluster_all[j].push_back(centroid[1]);
          centroid_cluster_all[j].push_back(centroid[2]);
          std::cout << j << "th cluster's centroid co-ords: " << centroid[0] << " " << centroid[1] << " " << centroid[2] << std::endl;
          

          visualization_msgs::msg::Marker marker_individual;
          // Set the frame ID and timestamp.  See the TF tutorials for information on these.
          marker_individual.header = msg_ptr->header;

          // Set the namespace and id for this marker.  This serves to create a unique ID
          // Any marker sent with the same namespace and id will overwrite the old one
          marker_individual.ns = "cluster";
          marker_individual.id = j;

          // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
          marker_individual.type = shape;
          //marker_individual.type = 9;
          //marker_individual.text = j;

          // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
          marker_individual.action = visualization_msgs::msg::Marker::ADD;

          // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
          marker_individual.pose.position.x = (min_pt_cluster[0] + max_pt_cluster[0])/2;
          marker_individual.pose.position.y = (min_pt_cluster[1] + max_pt_cluster[1])/2;
          marker_individual.pose.position.z = (min_pt_cluster[2] + max_pt_cluster[2])/2;
          marker_individual.pose.orientation.x = 0.0;
          marker_individual.pose.orientation.y = 0.0;
          marker_individual.pose.orientation.z = 0.0;
          marker_individual.pose.orientation.w = 1.0;

          // Set the scale of the marker -- 1x1x1 here means 1m on a side
          marker_individual.scale.x = max_pt_cluster[0] - min_pt_cluster[0];
          marker_individual.scale.y = max_pt_cluster[1] - min_pt_cluster[1];
          marker_individual.scale.z = max_pt_cluster[2] - min_pt_cluster[2];

          // Set the color -- be sure to set alpha to something non-zero!
          
          marker_individual.color.r = 1.0f;
          marker_individual.color.g = 1.0f;
          marker_individual.color.b = 1.0f;
          marker_individual.color.a = 0.5;
          
          
          marker_individual.lifetime.sec = 0;
          marker_individual.lifetime.nanosec = 0;

          marker_all.markers.push_back(marker_individual);
          

          
          // See if current cluster is present in the FOV of the robot and find angle of corner of AABB which is in FOV
          
          float angle_individual_radian;
          int is_obstacle_detected_in_fov = detect_obstacle(min_pt_cluster[0], min_pt_cluster[1], max_pt_cluster[0], max_pt_cluster[1], &angle_individual_radian);
          if(is_obstacle_detected_in_fov == 1) {
            angle_corner_radian_all[j].push_back(angle_individual_radian);
            std::cout << "Cluster # " << j << " obstacle detected in FOV: " << is_obstacle_detected_in_fov << " is at centroid angle: " << centroid_direction_array[j][0]*180/M_PI << " and corner angle: " << angle_corner_radian_all[j][0]*180/M_PI  <<std::endl;
            send(clientSock, &angle_individual_radian, sizeof(angle_individual_radian), 0);
            }
            // bzero(socket_buffer, 1000);
            // strcpy(socket_buffer, "test");
            // int n = write(clientSock, socket_buffer, strlen(socket_buffer));
            // std::cout << "Write Confirmation code  " << n << std::endl;            
            // shutdown(clientSock);
            
          else {
            float no_obs_angle_rad = 456.78;
            std::cout << "Cluster # " << j << " obstacle NOT detected in FOV: " << is_obstacle_detected_in_fov << " is at centroid angle: " << centroid_direction_array[j][0]*180/M_PI <<std::endl;
            send(clientSock, &no_obs_angle_rad, sizeof(no_obs_angle_rad), 0);}
          



          // std::cout << "Min point vector: " << min_pt_cluster_all[j][0] << std::endl;
          // std::cout << "Max x: " << max_pt_cluster[0] << std::endl;
          // std::cout << "Max y: " << max_pt_cluster[1] << std::endl;
          // std::cout << "Max z: " << max_pt_cluster[2] << std::endl;
          // std::cout << "Min x: " << min_pt_cluster[0] << std::endl;
          // std::cout << "Min y: " << min_pt_cluster[1] << std::endl;
          // std::cout << "Min z: " << min_pt_cluster[2] << std::endl;


          // sensor_msgs::msg::PointCloud2 ros_processed_pcl2; //Declaring a pointer using new was working but gave deprecated warning
          // pcl::toROSMsg(*cloud_cluster, ros_processed_pcl2);
          // publisher_pcl->publish(ros_processed_pcl2);        

          // std::stringstream ss;
          // ss << "cloud_cluster_" << j << ".pcd";
          // writer.write<pcl::PointXYZ> (ss.str (), *cloud_cluster, false); //*
          j++;
        }

        sensor_msgs::msg::PointCloud2 ros_processed_pcl2; //Declaring a pointer using new was working but gave deprecated warning
        pcl::toROSMsg(*cloud_cluster_all, ros_processed_pcl2);
        publisher_pcl->publish(ros_processed_pcl2);
        //marker_all.markers.push_back(fov_marker_pts);
        publisher_marker_array->publish(marker_all);
      }
      else {
        // Non-ground points not detected. Need to indicate this to Python script
        float no_obs_angle_rad = 456.78;
        std::cout << "Non-ground points not detected, sending 456.78" << std::endl;
        send(clientSock, &no_obs_angle_rad, sizeof(no_obs_angle_rad), 0);}

      //Publish data back to ROS2 for visualization
      // sensor_msgs::msg::PointCloud2 ros_processed_pcl2; //Declaring a pointer using new was working but gave deprecated warning
      // pcl_conversions::fromPCL(*cloud_filtered_ptr, ros_processed_pcl2_ptr);
      //pcl_conversions::fromPCL(*cloud_xyz_ptr, ros_processed_pcl2)
      // pcl::toROSMsg(*cloud_cluster, ros_processed_pcl2);

      // RCLCPP_INFO(this->get_logger(), "Process ROS2 PCL2, width is: '%d'", ros_processed_pcl2.width); //
      // publisher_pcl->publish(ros_processed_pcl2);
      std::cout << "Reached callback end in Private" << std::endl;
      // std::cin.ignore();
      
    }
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_vlp16;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_pcl;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher_marker_array;
  

};

int main(int argc, char * argv[])
{
  
  //Socket implementation ref: https://realpython.com/python-sockets/#echo-client, https://stackoverflow.com/questions/20732980/how-to-use-socket-with-a-python-client-and-a-c-server
  //int serverSock=socket(AF_INET, SOCK_STREAM, 0);
  //sockaddr_in serverAddr;
  //serverAddr.sin_family = AF_INET;
  //serverAddr.sin_port = SERVER_PORT;
  //serverAddr.sin_addr.s_addr = inet_addr("127.0.0.1");
  /* bind (this socket, local address, address length)
     bind server socket (serverSock) to server address (serverAddr).  
     Necessary so that server can use a specific port */ 
  //bind(serverSock, (struct sockaddr*)&serverAddr, sizeof(struct sockaddr));
  // wait for a client
  /* listen (this socket, request queue length) */
  //std::cout << "Waiting to listen on server socket" << std::endl;
  //listen(serverSock,1);
  //std::cout << "Listen complete" << std::endl;
  //clientSock=accept(serverSock,(struct sockaddr*)&clientAddr, &sin_size);
  //std::cout << "Socket connected" << std::endl;
  
  
  
  
  // receive a message from a client
  // read(clientSock, socket_buffer, 1000); // no contains the no. of bytes received; Ref: https://www.ibm.com/docs/en/zos/2.1.0?topic=functions-read-read-from-file-socket
  // std::cout << "Server received:  " << socket_buffer << std::endl;
  
  // strcpy(socket_buffer, "test");
  // write(clientSock, socket_buffer, strlen(socket_buffer));
      

  
  std::cout << "Inside main function now" << std::endl;
  rclcpp::init(argc, argv);
  std::cout << "RCLCPP init complete" << std::endl;
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  std::cout << "RCLCPP spin line executed" << std::endl;
  rclcpp::shutdown();
  std::cout << "RCLCPP shutdown completed" << std::endl;
  return 0;
}
