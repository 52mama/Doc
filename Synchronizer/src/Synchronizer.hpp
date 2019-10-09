#include <ros/ros.h>
#include <ros/console.h>

#include <std_msgs/Header.h>
#include <std_msgs/Float32.h>

#include <vector>
#include <iostream>

using namespace std;

template<class SUB_TYPE>
class Synchronizer
{

private:

    std::vector<ros::Subscriber> subs;
    std::vector<> subs_call_back;

    ros::Subscriber main_sub;
     main_sub_call_back;

    template<class SUB_TYPE>
    void main_call_back(SUB_TYPE in);

public:
    template<class SUB_TYPE>
    Synchronizer();
    ~Synchronizer();

};





template<class SUB_TYPE>
void Synchronizer::main_call_back(SUB_TYPE in)
{

}


template<class SUB_TYPE>
Synchronizer::Synchronizer(  )
{

    parameters = new Parameter( argv[1] );

    ros::init(argc, argv, "pc_process" );
    ros::NodeHandle nh;
    
    sub_point_cloud_ = nh.subscribe( "/pandar_points" , 5, &Synchronizer::point_cb, this);
    pub_bounding_boxs_ = nh.advertise<jsk_recognition_msgs::BoundingBoxArray>("/detected_bounding_box", 5);
    ros::spin();
    
}



Synchronizer::~Synchronizer() 
{

}

void Synchronizer::point_cb(const sensor_msgs::PointCloud2ConstPtr &in_cloud_ptr  )
{

    
    // time
    ROS_INFO( " Node:EuCluster.       seq: %d           recieved. " , in_cloud_ptr->header.seq );
    
    time_begin = ros::Time::now();
    

    pcl::PointCloud<pcl::PointXYZ>::Ptr current_pc_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    
    point_cloud_header_ = in_cloud_ptr->header;
    
    pcl::fromROSMsg(*in_cloud_ptr, *current_pc_ptr);
    
    std::vector<int> mapping;
    pcl::removeNaNFromPointCloud(*current_pc_ptr, *current_pc_ptr, mapping);
    
    pass_throught(current_pc_ptr);
    voxel_filter(current_pc_ptr);
    
    //ground_removal(current_pc_ptr);
    segmentPlane(current_pc_ptr,current_pc_ptr,0.05);
    

    std::vector<Detected_Obj> global_obj_list;
    cluster_by_distance(current_pc_ptr, global_obj_list);


    jsk_recognition_msgs::BoundingBoxArray bbox_array;

    for (size_t i = 0; i < global_obj_list.size(); i++)
    {
        bbox_array.boxes.push_back(global_obj_list[i].bounding_box_);
    }
    bbox_array.header = point_cloud_header_;

    
    ROS_INFO( " Node:EuCluster.       seq: %d           published. " , in_cloud_ptr->header.seq );
    time_end = ros::Time::now();
    time_data.data = (time_end - time_begin).toSec()*10000 ;
    pub_time.publish( time_data );

    pub_bounding_boxs_.publish(bbox_array);

}
