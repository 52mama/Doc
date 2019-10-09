
#include <ros/ros.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>

#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#include <pcl/segmentation/conditional_euclidean_clustering.h>

#include <pcl/common/common.h>
#include <pcl/segmentation/extract_clusters.h>

#include <pcl/search/organized.h>
#include <pcl/search/kdtree.h>

#include <std_msgs/Header.h>

#include <jsk_recognition_msgs/BoundingBox.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>

#include <sensor_msgs/PointCloud2.h>
#include <pcl/filters/crop_box.h>

#include <ctime>


#include <std_msgs/Float32.h>
#include <ros/console.h>

#include "/home/hrt19d/parameter/Parameters_Reader.hpp"

#include <chrono>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/kdtree/kdtree.h>
#include <Eigen/Core>


class EuClusterCore
{

private:

  struct Detected_Obj
  {
    jsk_recognition_msgs::BoundingBox bounding_box_;

    pcl::PointXYZ min_point_;
    pcl::PointXYZ max_point_;
    pcl::PointXYZ centroid_;
  };

  ros::Subscriber sub_point_cloud_;

  ros::Publisher pub_bounding_boxs_;
  ros::Publisher pub_noground;
  
  ros::Publisher pub_marker;

  std_msgs::Header point_cloud_header_;

  void segmentPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,pcl::PointCloud<pcl::PointXYZ>::Ptr obj_cloud,
                  float distanceThreshold);

  void voxel_grid_filer(pcl::PointCloud<pcl::PointXYZ>::Ptr in, pcl::PointCloud<pcl::PointXYZ>::Ptr out, double leaf_size);

  void cluster_by_distance(pcl::PointCloud<pcl::PointXYZ>::Ptr in_pc, std::vector<Detected_Obj> &obj_list);

  void cluster_segment(pcl::PointCloud<pcl::PointXYZ>::Ptr in_pc,
                       double cluster_distance, 
                       int cluster_min_size , 
                       int cluster_max_size, 
                       std::vector<Detected_Obj> & obj_list);

  void pass_throught(pcl::PointCloud<pcl::PointXYZ>::Ptr &current_pc_ptr);
  void voxel_filter(pcl::PointCloud<pcl::PointXYZ>::Ptr &current_pc_ptr);
  void ground_removal(pcl::PointCloud<pcl::PointXYZ>::Ptr &current_pc_ptr);


  void point_cb(const sensor_msgs::PointCloud2ConstPtr &in_cloud_ptr);

  void publish_cloud(const ros::Publisher &in_publisher,
                     const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_to_publish_ptr,
                     const std_msgs::Header &in_header);
  bool _____(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

  inline double abs_D(double x);
  double angle_region;
  ros::Publisher pub_time;
  ros::Time time_begin;
  ros::Time time_end;
  std_msgs::Float32 time_data;

  Parameter *parameters ;

public:


  EuClusterCore(int argc   , char **argv,double angle_region_);
  ~EuClusterCore();
};
