#include "euclidean_cluster_core.h"
#include <iostream>
#include </usr/include/eigen3/Eigen/Dense>


#include <math.h> 


using namespace std;



EuClusterCore::EuClusterCore(int argc   , char **argv,double angle_region_):angle_region(angle_region_)
{

    parameters = new Parameter( argv[1] );
    

    ros::init(argc, argv, "pc_process" );
    ros::NodeHandle nh;
    
    sub_point_cloud_ = nh.subscribe( "/pandar_points" , 5, &EuClusterCore::point_cb, this);
    pub_bounding_boxs_ = nh.advertise<jsk_recognition_msgs::BoundingBoxArray>("/detected_bounding_box", 5);
    pub_noground = nh.advertise<sensor_msgs::PointCloud2>("/no_ground", 5);
    pub_time = nh.advertise<std_msgs::Float32>( "/EuCluster/time" , 5); 
    ros::spin();
    
}



EuClusterCore::~EuClusterCore() 
{
    delete parameters;
}
/**
*@brief:发布点云
*@prarm：in_publisher 发布者 in_cloud_publish_ptr pcl格式点云 in_header 输入点云的头信息
*@return void
*/

void EuClusterCore::publish_cloud(const ros::Publisher &in_publisher,
                                  const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_to_publish_ptr,
                                  const std_msgs::Header &in_header)
{
    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(*in_cloud_to_publish_ptr, cloud_msg);
    cloud_msg.header = in_header;
    in_publisher.publish(cloud_msg);
}


inline double EuClusterCore::abs_D(double x)
{
  return x>0?x:-x;
}




void EuClusterCore::cluster_segment(pcl::PointCloud<pcl::PointXYZ>::Ptr in_pc,
                                    double cluster_distance, 
                                    int cluster_min_size, 
                                    int cluster_max_size, 
                                    std::vector<Detected_Obj> &obj_list)
{


    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    std::vector<pcl::PointIndices> local_indices;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_2d(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud(*in_pc, *cloud_2d);
    for (size_t i = 0; i < cloud_2d->points.size(); i++)
    {
        cloud_2d->points[i].z = 0;
    }

    pcl::EuclideanClusterExtraction<pcl::PointXYZ> euclid;

    euclid.setInputCloud(cloud_2d);
    euclid.setClusterTolerance(cluster_distance);
    euclid.setMinClusterSize(cluster_min_size);
    euclid.setMaxClusterSize(cluster_max_size);
    euclid.setSearchMethod(tree);
    euclid.extract(local_indices);


    
    for (size_t i = 0; i < local_indices.size(); i++)
    {
        // the structure to save one detected object
        Detected_Obj obj_info;

        float min_x = std::numeric_limits<float>::max();
        float max_x = -std::numeric_limits<float>::max();
        float min_y = std::numeric_limits<float>::max();
        float max_y = -std::numeric_limits<float>::max();
        float min_z = std::numeric_limits<float>::max();
        float max_z = -std::numeric_limits<float>::max();

        for (auto pit = local_indices[i].indices.begin(); pit != local_indices[i].indices.end(); ++pit)
        {
            //fill new colored cluster point by point
            pcl::PointXYZ p;
            p.x = in_pc->points[*pit].x;
            p.y = in_pc->points[*pit].y;
            p.z = in_pc->points[*pit].z;

            obj_info.centroid_.x += p.x;
            obj_info.centroid_.y += p.y;
            obj_info.centroid_.z += p.z;

            if (p.x < min_x)
                min_x = p.x;
            if (p.y < min_y)
                min_y = p.y;
            if (p.z < min_z)
                min_z = p.z;
            if (p.x > max_x)
                max_x = p.x;
            if (p.y > max_y)
                max_y = p.y;
            if (p.z > max_z)
                max_z = p.z;
        }


	if( max_x-min_x > parameters->Eu_Cluster->Cone_Length_Judgement.x_span || 
        max_y-min_y > parameters->Eu_Cluster->Cone_Length_Judgement.y_span || 
        max_z-min_z > parameters->Eu_Cluster->Cone_Length_Judgement.z_span   )
	{
        continue;
    }
    else if( max_z-min_z < max_y-min_y || max_z-min_z < max_x-min_x )
    {   
        continue ;    
    }
	else
    {
		
        //min, max points
        obj_info.min_point_.x = min_x;
        obj_info.min_point_.y = min_y;
        obj_info.min_point_.z = min_z;

        obj_info.max_point_.x = max_x;
        obj_info.max_point_.y = max_y;
        obj_info.max_point_.z = max_z;

        //calculate centroid, average
        if (local_indices[i].indices.size() > 0)
        {
            obj_info.centroid_.x /= local_indices[i].indices.size();
            obj_info.centroid_.y /= local_indices[i].indices.size();
            obj_info.centroid_.z /= local_indices[i].indices.size();
        }

        //calculate bounding box
        double length_ = obj_info.max_point_.x - obj_info.min_point_.x;
        double width_ = obj_info.max_point_.y - obj_info.min_point_.y;
        double height_ = obj_info.max_point_.z - obj_info.min_point_.z;

        obj_info.bounding_box_.header = point_cloud_header_;

        obj_info.bounding_box_.pose.position.x = obj_info.min_point_.x + length_ / 2;
        obj_info.bounding_box_.pose.position.y = obj_info.min_point_.y + width_ / 2;
        obj_info.bounding_box_.pose.position.z = obj_info.min_point_.z + height_ / 2;

        obj_info.bounding_box_.dimensions.x = ((length_ < 0) ? -1 * length_ : length_);
        obj_info.bounding_box_.dimensions.y = ((width_ < 0) ? -1 * width_ : width_);
        obj_info.bounding_box_.dimensions.z = ((height_ < 0) ? -1 * height_ : height_);

        obj_list.push_back(obj_info);
}
    }
}
/**
*@beief:按距离聚类（尽可能保证聚类适应更多情况）
*@prarm in_pc-待聚类的点云 obj_list-被聚类出的object列表
*@return void
*/

void EuClusterCore::cluster_by_distance(pcl::PointCloud<pcl::PointXYZ>::Ptr in_pc, std::vector<Detected_Obj> &obj_list)
{
    //cluster the pointcloud according to the distance of the points using different thresholds (not only one for the entire pc)
    //in this way, the points farther in the pc will also be clustered

    //0 => 0-15m d=0.5
    //1 => 15-30 d=1
    //2 => 30-45 d=1.6
    //3 => 45-60 d=2.1
    //4 => >60   d=2.6

    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>  segment_pc_array( parameters->Eu_Cluster->EuCluster_Parameters.size() );

    for (size_t i = 0; i < segment_pc_array.size(); i++)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr tmp(new pcl::PointCloud<pcl::PointXYZ>);
        segment_pc_array[i] = tmp;
    }

    for (size_t i = 0; i < in_pc->points.size(); i++)
    {
        pcl::PointXYZ current_point;
        current_point.x = in_pc->points[i].x;
        current_point.y = in_pc->points[i].y;
        current_point.z = in_pc->points[i].z;

        float origin_distance = sqrt(pow(current_point.x, 2) + pow(current_point.y, 2));
        
        // 如果点的距离大于120m, 忽略该点
        if (origin_distance >=  parameters->Eu_Cluster->Max_distance )
        {
            continue;
        }
        

        // 注意：json中的距离分段必须递增
        for(size_t i=0 ; i < parameters->Eu_Cluster->EuCluster_Parameters.size() ;i++)
        {
            if( origin_distance < parameters->Eu_Cluster->EuCluster_Parameters[i].seg_distance_max
             && origin_distance > parameters->Eu_Cluster->EuCluster_Parameters[i].seg_distance_min )
            {
                segment_pc_array[i]->points.push_back(current_point);
            }
        }

    }

    for (size_t i = 0; i < segment_pc_array.size(); i++)
    {
        cluster_segment(segment_pc_array[i], 
                        parameters->Eu_Cluster->EuCluster_Parameters[i].cluster_distance,
                        parameters->Eu_Cluster->EuCluster_Parameters[i].cluster_min_size,
                        parameters->Eu_Cluster->EuCluster_Parameters[i].cluster_max_size, 
                        obj_list);
    }
}


void EuClusterCore::pass_throught(pcl::PointCloud<pcl::PointXYZ>::Ptr &current_pc_ptr)
{
    
    pcl::ExtractIndices<pcl::PointXYZ> passthrough;
	pcl::PointIndices indices;
	passthrough.setInputCloud(current_pc_ptr);
    #pragma omp for 
    for(size_t i=0;i<current_pc_ptr->points.size();i++)
	{
		if(  ! ( current_pc_ptr->points[i].z > parameters->Pass_Through->z_min && current_pc_ptr->points[i].z < parameters->Pass_Through->z_max ) )	
        {	continue;	}
		if(  ! ( current_pc_ptr->points[i].y > parameters->Pass_Through->y_min && current_pc_ptr->points[i].y < parameters->Pass_Through->y_max ) )	
        {	continue;	}
		if(  ! ( current_pc_ptr->points[i].x > parameters->Pass_Through->x_min  && current_pc_ptr->points[i].x < parameters->Pass_Through->x_max ) )	
        {	continue;	}
		indices.indices.push_back(i);
	}
	passthrough.setIndices(boost::make_shared<pcl::PointIndices>(indices));
	passthrough.setNegative(false);
	passthrough.filter(*current_pc_ptr);  
}

void EuClusterCore::voxel_filter(pcl::PointCloud<pcl::PointXYZ>::Ptr &current_pc_ptr)
{
    
    pcl::VoxelGrid<pcl::PointXYZ> filter;
    filter.setInputCloud(current_pc_ptr);
    filter.setLeafSize( parameters->Voxel_Filter->size, 
                        parameters->Voxel_Filter->size,
                        parameters->Voxel_Filter->size );
    filter.filter(*current_pc_ptr);

}



inline double abs_D(double a)
{
    return a>0?a:-a;
}

inline double distance_point_pane(double a,double b,double c,double d,pcl::PointXYZ P)
{
    return abs_D( a*P.x + b*P.y + c*P.z + d ) / sqrt(a*a+b*b+c*c);
}
inline void setPointXYZ(pcl::PointXYZ &A,pcl::PointXYZ &B)
{
    A.x = B.x;
    A.y = B.y;
    A.z = B.z;
}
#define PI 3.1415926

void EuClusterCore::segmentPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                  pcl::PointCloud<pcl::PointXYZ>::Ptr obj_cloud,
                  float distanceThreshold)
{
    pcl::ModelCoefficients::Ptr cofficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(distanceThreshold);
    seg.setInputCloud(cloud);
    seg.segment(*inliers,*cofficients);

    if(inliers->indices.size() == 0)
        std::cout << "Could not estimate a planar model for the given dataset." << std::endl;

    pcl::PointCloud<pcl::PointXYZ>::Ptr ground_cloud(new pcl::PointCloud<pcl::PointXYZ>());


    for(int index : inliers->indices)
        ground_cloud->points.push_back(cloud->points[index]);

    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*obj_cloud);

    
    
    publish_cloud(pub_noground,obj_cloud,point_cloud_header_);

}
void EuClusterCore::ground_removal(pcl::PointCloud<pcl::PointXYZ>::Ptr &current_pc_ptr)
{
    vector< vector< pcl::PointXYZ >> points{ parameters->Ground_Removal->Sector_Num , 
                                             vector<pcl::PointXYZ>( parameters->Ground_Removal->Pie_Num , pcl::PointXYZ{0,0,99} ) 
                                           }  ;

    vector< double > tangent_Pie;
    for(int i = 1 ; i <= parameters->Ground_Removal->Sector_Num ; i++ )
    {
        tangent_Pie.push_back( tan( PI/2 - PI*i/parameters->Ground_Removal->Sector_Num ) );
    }

    int index;
    for( size_t i = 0  ; i < current_pc_ptr->points.size() ; i++)
    {
        index = (int) (sqrt( ( current_pc_ptr->points[i].x*current_pc_ptr->points[i].x + current_pc_ptr->points[i].y*current_pc_ptr->points[i].y ) )/0.3);
        if( index > parameters->Ground_Removal->Pie_Num -1 )   continue ;

        for( size_t j = 0 ; j<tangent_Pie.size() ; j++ )
        {
            if( tangent_Pie[j]*current_pc_ptr->points[i].x < current_pc_ptr->points[i].y )
                if( points[j][index].z > current_pc_ptr->points[i].z )   //判断是否更低
                    setPointXYZ( points[j][index] , current_pc_ptr->points[i] );
        }
    }
    
    double x_2{0},y_2{0},x_y{0},x_z{0},y_z{0},x{0},z{0},y{0},sum{0}; 
    for(size_t i = 0 ; i < points.size()  ; i++ ) 
    {
        for(size_t j = 0 ; j < points[i].size() ; j++ )
        {
            if( points[i][j].z ==99 )   continue ;
            sum +=1 ;
            x_2 += points[i][j].x * points[i][j].x ;
            y_2 += points[i][j].y * points[i][j].y ;
            x_y += points[i][j].y * points[i][j].x ;
            y_z += points[i][j].y * points[i][j].z ;
            x_z += points[i][j].x * points[i][j].z ;
            x += points[i][j].x;
            y += points[i][j].y;
            z += points[i][j].z;
        }
    }

    Eigen::MatrixXf mat_1(3,3),mat_2(3,1);
    mat_1 << x_2 , x_y , x , x_y , y_2 , y , x , y , sum;
    mat_2 << x_z , y_z , z;
    Eigen::MatrixXf mat_res = mat_1.inverse() * mat_2;

    // cout<<mat_res<<endl;

    

    pcl::PointIndices indices;
    // 取出地面附近的点
    for(size_t i = 0; i < current_pc_ptr->points.size(); i++)
    {
        if( parameters->Ground_Removal->threshold < distance_point_pane( mat_res.data()[0] , mat_res.data()[1] , -1 , mat_res.data()[2] , current_pc_ptr->points[i])  )
        {
            indices.indices.push_back(i);
        }
    }

    
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud (current_pc_ptr);
    extract.setIndices (boost::make_shared<pcl::PointIndices>(indices));
    extract.setNegative (false);//如果设为true,可以提取指定index之外的点云
    extract.filter (*current_pc_ptr);
}

void EuClusterCore::point_cb(const sensor_msgs::PointCloud2ConstPtr &in_cloud_ptr  )
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
