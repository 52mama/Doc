
// 1.85525   0.18   -0.87247   -0.003
#include <ros/ros.h> 

#include <pcl/PCLPointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>

#include <visualization_msgs/Marker.h>

#include <pcl/keypoints/harris_3d.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/filters/voxel_grid.h>

#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>  

#include "opencv2/highgui/highgui.hpp"//画点
#include "opencv2/imgproc/imgproc.hpp"




#include "cv.h"    //绘制文字
#include "highgui.h" 

#include <string>//int转string
#include <iostream>

#include <vector>

#include <pthread.h>//多线程

#include <math.h>


#include <fstream>

#include <pcl/features/normal_3d.h>//法线
#include <pcl/features/principal_curvatures.h>//曲率

#include <algorithm>

#include <stdlib.h>

using namespace std;
using namespace cv;


std::vector<cv::Point> flag_point;
int x_mouse,y_mouse;
ros::Publisher pub;



void calibration(); 
void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& input);
void gradient_descent();
void pcl_lidar(const sensor_msgs::PointCloud2ConstPtr& input);
void cv_onMouseHandle(int event,int x,int y,int flags, void *param);
void cv_camera();

float x_max;
float x_min;
float y_max;
float y_min;
float z_min;
float z_max;

float x__=0;
float y__=0;
float z__=0;
float O__=0;

std::ofstream out_lidar("点云.txt");
std::ofstream out_camera("像素.txt");

std::vector<cv::Point> points;
std::vector<cv::Scalar> points_color;

bool done=true;

std::vector<std::vector<double>> M_mul(const std::vector<std::vector<double>> &left,const std::vector<std::vector<double>> &right)
{
    std::vector<std::vector<double>> m;
    for(size_t i=0;i<left.size();i++)
    {
	std::vector<double> m_;
	for(size_t j=0;j<right[0].size();j++)
	{
	    double sum=0;
	    for(size_t k=0;k<right.size();k++)
	    {
    		sum+=left[i][k]*right[k][j];
	    }
	    m_.push_back(sum);
	}
	m.push_back(m_);
    }
    return m;
}



void build_marker(visualization_msgs::Marker &marker,float position_x,float position_y,float position_z)
{
    marker.header.frame_id = "rslidar";
    marker.header.stamp = ros::Time::now();
    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = "basic_shapes";
    marker.id = 0;
    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
    marker.type = visualization_msgs::Marker::CUBE;
    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    marker.action = visualization_msgs::Marker::ADD;
    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    marker.pose.position.x = position_x;
    marker.pose.position.y = position_y;
    marker.pose.position.z = position_z;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.03;
    marker.scale.y = 0.03;
    marker.scale.z = 0.03;
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;
    marker.lifetime = ros::Duration();
}




void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& input) 
{
    

    //ros点云转pcl点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_input(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_output(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg (*input,*cloud_input);
    std::vector<int> mapping;
    pcl::removeNaNFromPointCloud(*cloud_input, *cloud_input, mapping);
    
    pcl::ExtractIndices<pcl::PointXYZ> passthrough;
    pcl::PointIndices indices;
    passthrough.setInputCloud(cloud_input);
    for(size_t i=0;i<cloud_input->points.size();i++)
    {

	    if(   cloud_input->points[i].y > y_max || cloud_input->points[i].y < y_min ) 	{	continue;	}
	    if(   cloud_input->points[i].x > x_max || cloud_input->points[i].x < x_min ) 	{	continue;	}
	    if(   cloud_input->points[i].z < z_min || cloud_input->points[i].z > z_max  ) 	{	continue;	}
	    indices.indices.push_back(i);
	    out_lidar<<cloud_input->points[i].x<<" "<<cloud_input->points[i].y<<" "<<cloud_input->points[i].z<<std::endl;
    }
    out_lidar.close();
    passthrough.setNegative(false);
    passthrough.setIndices(boost::make_shared<pcl::PointIndices>(indices));		
    passthrough.filter(*cloud_input); 

    
    cout<<"asd"<<endl;
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg (*cloud_input,output);
    output.header.frame_id = input->header.frame_id;
	
    //发布
    pub.publish(output);

}
inline vector<double> undistorted(double k1,double k2,double p1,double p2,double x,double y)
{
    double r2 = x*x + y*y ;
    vector<double>  x_y;
    double aaa = 1 + k1*r2 + k2*r2*r2;
    x_y.push_back( x*aaa + 2*p1*x*y + p2*(r2+2*x*x) );
    x_y.push_back( y*aaa + p1*(r2+2*y*y) + 2*p2*x*y );
    return x_y;
}

void cloud_cb_1(const sensor_msgs::PointCloud2ConstPtr& input) 
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_input(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg (*input, *cloud_input);
    std::vector<int> mapping; 
    pcl::removeNaNFromPointCloud(*cloud_input, *cloud_input, mapping);
    points.clear();
    points_color.clear();

    vector<double> x_y_undistorted;

    for(size_t  i=0;i<cloud_input->points.size();i++)
    {
	    if(cloud_input->points[i].x < 0)	continue;
	    
        // 老版手动旋转平移
        float x_r=cos(O__)*(cloud_input->points[i].x+x__)-sin(O__)*(cloud_input->points[i].y+y__);
	    float y_r=sin(O__)*(cloud_input->points[i].x+x__)+cos(O__)*(cloud_input->points[i].y+y__);
        float z = cloud_input->points[i].z+z__;

        x_y_undistorted = undistorted(-0.313640,0.079529,-0.001175,0.000956,y_r/x_r,z/x_r);

        x_r = x_y_undistorted[0];
        y_r = x_y_undistorted[1];

        // std::vector<std::vector<double>> XYZ = M_mul(
        
        // {{1.080288125493850 ,  0.385866330629433   ,0.468176704608606  , },
        // {-0.154952556473543   ,1.087544876377362   ,0.393346926660157  , },
        // {-0.104275602707729  ,-0.145096762037743,   0.358977001016778  ,  },
        // {-0.000000000000000  ,-0.000000000000000  ,-0.000000000000000  }} ,

        // {{cloud_input->points[i].x},{cloud_input->points[i].y},{cloud_input->points[i].z},{1}}
        // );

        // // prejection
	    // std::vector<std::vector<double>> UV = M_mul({{-696.250427 ,0.000000, 954.788616 },
        //                                             {0.000000 ,-879.107056, 548.086666 },
        //                                             {0.000000 ,0.000000 ,1.000000 } },
        //                                              {{y_r},{z},{x_r}}  );
        
        // camera matrix
        // -0.0400001   3.50177e-09   -0.08   4.60912e-09
	    


        // std::vector<std::vector<double>> UV = M_mul({{-971.057507 ,0.000000 ,947.979129},
        //                                             {0.000000, -974.701312 ,542.246776},
        //                                             {0.000000, 0.000000 ,1.000000}},
        //                                             {{y_r},{z},{x_r}}  );
        // double x=UV[0][0]/x_r ;
	    // double y=UV[1][0]/x_r ;


        double x=-971.057507*x_r + 947.979129  ;
        double y=-974.701312*y_r + 542.246776 ;
        
	    points.push_back(  cv::Point( x,y )  );

				      
	    float dis=cloud_input->points[i].x*cloud_input->points[i].x+
		  cloud_input->points[i].y*cloud_input->points[i].y+
		  cloud_input->points[i].z*cloud_input->points[i].z;
	    points_color.push_back( cv::Scalar(255,255,255)  );
    }
}


void* calibration(void* param)
{
    ros::NodeHandle nh;
    ros::Subscriber sub=nh.subscribe("input",1,cloud_cb_1); 
    //pub = nh.advertise<visualization_msgs::Marker>("calibration_lidar_flag", 1);
    pub = nh.advertise<sensor_msgs::PointCloud2>("calibration_lidar_flag", 1);
    while(done)
    {
	    ros::spinOnce();
    }

}

void cv_onMouseHandle(int event,int x,int y,int flags, void *param)
{
    //cv::Mat* image = (cv::Mat*) param;
    switch (event)
    {
    case CV_EVENT_MOUSEMOVE :
			    x_mouse=x;
			    y_mouse=y;
			    break;
    case CV_EVENT_LBUTTONDOWN:
			    flag_point.push_back(cv::Point(x,y));
			    std::cout<<"pixel: x:"<<x<<"  y:"<<y<<std::endl;
			    out_camera<<x<<" "<<y<<std::endl;
			    break;
    }
}
void* cv_camera(void* args)
{
    
    cv::VideoCapture cap;
    cap.open(0); //打开摄像头
                // cap.set(CV_CAP_PROP_FRAME_WIDTH, 1920.0);
    // cap.set(CV_CAP_PROP_FRAME_HEIGHT, 1024.0);
    if(!cap.isOpened())	return 0 ;
    cv::Mat image;
    
    //cv::namedWindow("frame", CV_WINDOW_NORMAL);
    //cv::setWindowProperty("frame", CV_WND_PROP_FULLSCREEN, CV_WINDOW_FULLSCREEN);
    
    Mat cameraMatrix = Mat::eye(3, 3, CV_64F);
    cameraMatrix.at<double>(0, 0) = 971.057507 ;
    cameraMatrix.at<double>(0, 2) = 947.979129 ;
    cameraMatrix.at<double>(1, 1) = 974.701312 ;
    cameraMatrix.at<double>(1, 2) = 542.246776 ;
 
    Mat distCoeffs = Mat::zeros(5, 1, CV_64F);
    distCoeffs.at<double>(0, 0) = -0.313640;
    distCoeffs.at<double>(1, 0) = 0.07952 ;
    distCoeffs.at<double>(2, 0) = -0.001175 ;
    distCoeffs.at<double>(3, 0) = 0.000956;
    distCoeffs.at<double>(4, 0) = 0;

    Mat view, rview, map1, map2;
    Size imageSize(1920,1024);

    initUndistortRectifyMap(cameraMatrix, distCoeffs, Mat(),
        getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, imageSize, 1, imageSize, 0),
        imageSize, CV_16SC2, map1, map2);
    


    int width = cap.get(CV_CAP_PROP_FRAME_WIDTH);  		//帧宽度
    int height = cap.get(CV_CAP_PROP_FRAME_HEIGHT); 		//帧高度
    int frameRate = cap.get(CV_CAP_PROP_FPS);  			//帧率 x framess
    int totalFrames = cap.get(CV_CAP_PROP_FRAME_COUNT); 	//总帧数
    
    
    std::cout<<"视频宽度="<<width<<std::endl;
    std::cout<<"视频高度="<<height<<std::endl;
    std::cout<<"视频总帧数="<<totalFrames<<std::endl;
    std::cout<<"帧率="<<frameRate<<std::endl;
    
    while(1)
    {
        cap>>image;//等价于cap.read(frame);
        
        if(image.empty())  break; 

	    //显示标定的点
        for(int i=0;i<points.size();i++)
	    {
	        circle(image, points[i] , 1 ,points_color[i],-1); 
	    }
        
        remap(image, image, map1, map2,INTER_LINEAR);

	    

	    //鼠标事件
	    cv::setMouseCallback(   "video",			//const char* window_name       窗口的名称
	    			cv_onMouseHandle,	//CvMouseCallback on_mouse      回调函数
	    			(void*)&image			//void* param CV_DEFAULT(NULL)) 传递到回调函数的？？图片？？,这里使用视频流
	    				);
    
	    //绘制文字 
	    std::string x_y=std::to_string(x_mouse)+":"+std::to_string(y_mouse);
	    cv::putText(   image,				//Mat& img  图片
	    	   x_y , 				//const string& text   显示的文本
	    	   cv::Point(20,30), 			//Point org   坐标
	    	   CV_FONT_HERSHEY_COMPLEX, 		//int fontFace 字体
	    	   0.75, 				//double fontScale 大小
	    	   cv::Scalar(888), 			//Scalar color  颜色
	    	   1, 					//int thickness  笔画粗细
	    	   cv::LINE_8				//线型
	    	);

	    cv::imshow("video", image );

        
        cout<<x__<<"    "<<y__<<"     "<<z__<<"     "<<O__<<endl;

	    char key=cv::waitKey(1);
        if(key=='w')	x__+=0.01;
	    if(key=='s')	x__-=0.01;
	    if(key=='a')	y__+=0.01;
	    if(key=='d')	y__-=0.01;
	    if(key=='r')	z__+=0.01;
	    if(key=='f')	z__-=0.01;
	    if(key=='q')	O__+=0.001;
	    if(key=='e')	O__-=0.001;
    
            if(key=='z') 	
	    {
	        break;
	        done=false;
	    }
    }
    cap.release();
    cv::destroyAllWindows();
    
    return 0; 
    
}


int main(int argc, char** argv)
{
    ros::init (argc, argv, "calibration");
    
    int i = 1;
    
    // if(argc!=1+6)
    // {
	//     cout<<"params"<<endl;
	//     return 0;
    // }
    
    // x_min=atof( argv[1] );
    // x_max=atof( argv[2] );
    // y_min=atof( argv[3] );
    // y_max=atof( argv[4] );
    // z_min=atof( argv[5] );
    // z_max=atof( argv[6] );
    
    
    // //int i = 1;
    // if(argc!=1+4)
    // {
	//     cout<<"params"<<endl;
	//     return 0;
    // }
    // x__=atof( argv[1] );
    // y__=atof( argv[2] );
    // z__=atof( argv[3] );
    // O__=atof( argv[4] );
    
    // 1.77   -0.13   -0.919999   -0.037
    
    O__=-0.05;//-0.14882532700659806 ;            
    x__=0.02; 
    y__=-0.1;   
    z__=0.07;
    // 0.09    -0.04     0.11     -0.055
   
    
    pthread_t tids_1;
    pthread_t tids_2;
    //参数依次是：创建的线程id，线程参数，调用的函数，传入的函数参数
    if (pthread_create(&tids_1, NULL, cv_camera, NULL) != 0)
    {
	    cout << "pthread_create error: error_code" << endl;
    }
    if (pthread_create(&tids_2, NULL, calibration, NULL) != 0)
    {
	    cout << "pthread_create error: error_code="  << endl;
    }
    
    //等各个线程退出后，进程才结束，否则进程强制结束了，线程可能还没反应过来；
    pthread_exit(NULL);
    
    //calibration();
    
    return 0;
}




    /*
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    vg.setInputCloud(cloud_input);
    vg.setLeafSize(  0.02 ,  0.02  ,  0.02 );
    vg.filter(*cloud_input);
    */
    
    /*
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>); 
    tree->setInputCloud (cloud_input);
    
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance ( 0.08); 
    ec.setMinClusterSize (15);
    ec.setMaxClusterSize (200);	
    ec.setSearchMethod (tree);//设置点云的搜索机制 
    ec.setInputCloud (cloud_input); 
    ec.extract (cluster_indices);
    
    std::cout<<cluster_indices.size()<<std::endl;	
    */
    
    /*
    for(auto i=cluster_indices.begin();i!=cluster_indices.end();i++)
    {
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_temp(new pcl::PointCloud<pcl::PointXYZ>);
	for(auto j=i->indices.begin();j!=i->indices.end();j++)
	{
	   
	    cloud_temp->points.push_back(	cloud_input->points[ *j ]	);
	}
	*cloud_output+=*cloud_temp;
    }*/
    
    /*
    for(int i=0;i<cluster_indices.size();i++)
    {
	for(auto j=cluster_indices[i].indices.begin();j!=cluster_indices[i].indices.end();j++)
	{
	    out_lidar<<i<<" "<<cloud_input->points[ *j ].x<<" "<<cloud_input->points[ *j ].y<<" "<<cloud_input->points[ *j ].z<<std::endl;
	}
    }
    * */
    /*
    vector<PCURVATURE> curvatures= getModelCurvatures(cloud_input);
    sort(curvatures.begin(),curvatures.end(),big);
    
    for(size_t i=0; i<5 && i<curvatures.size();i++)
    {
	cloud_output->points.push_back(cloud_input->points[ curvatures[i].index ]);
    }
    */
    /*
    //欧式分割
    
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>); 
    tree->setInputCloud (cloud_input); 
    


    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec; 
    ec.setClusterTolerance ( 0.04); 
    ec.setMinClusterSize (200);
    ec.setMaxClusterSize (500);	
    ec.setSearchMethod (tree);//设置点云的搜索机制 
    ec.setInputCloud (cloud_input); 
    ec.extract (cluster_indices);
    
    
    
    cloud_output->width=0;
    cloud_output->height=1;
    
    
    double x_min,x_max;
    double y_min,y_max;
    double z_min,z_max;
    for(auto i=cluster_indices.begin();i!=cluster_indices.end();i++)
    {
	x_min = x_max =  cloud_input->points[*i->indices.begin ()].x ;
	y_min = y_max =  cloud_input->points[*i->indices.begin ()].y ;
	z_max = z_min =  cloud_input->points[*i->indices.begin ()].z ;
	bool is_target=true;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_temp(new pcl::PointCloud<pcl::PointXYZ>);
	for(auto j=i->indices.begin();j!=i->indices.end();j++)
	{
	    if( cloud_input->points[*j].x > x_max ) 	{ x_max = cloud_input->points[*j].x; }
	    if( cloud_input->points[*j].x < x_min ) 	{ x_min = cloud_input->points[*j].x; }
	    if( cloud_input->points[*j].y > y_max ) 	{ y_max = cloud_input->points[*j].y; }
	    if( cloud_input->points[*j].y < y_min ) 	{ y_min = cloud_input->points[*j].y; }
	    if( cloud_input->points[*j].z > z_max ) 	{ z_max = cloud_input->points[*j].z; }
	    if( cloud_input->points[*j].z < z_min ) 	{ z_min = cloud_input->points[*j].z; }
	    if( x_max - x_min >0.15  || 
		y_max - y_min >0.25  || 
		z_max - z_min > 0.25  
		          )
	    {
		is_target=false;
		break;
	    }
	    cloud_temp->points.push_back(	cloud_input->points[ *j ]	);
	}
	if(is_target)
	{
	    *cloud_output+=*cloud_temp;
	}
    }

    pcl::VoxelGrid<pcl::PointXYZ> vg;
    vg.setInputCloud(cloud_output);
    vg.setLeafSize(  0.01 ,  0.01  ,  0.01 );
    vg.filter(*cloud_output);

    //采样一致找标定板

    std::vector<int> inliers;
    pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr  model_p (new pcl::SampleConsensusModelPlane<pcl::PointXYZ> (cloud_output)); 
    pcl::RandomSampleConsensus<pcl::PointXYZ> ransac (model_p);
    ransac.setDistanceThreshold (0.02);   
    ransac.computeModel();               
    ransac.getInliers(inliers); 
    pcl::copyPointCloud<pcl::PointXYZ>(*cloud_output, inliers, *cloud_output);
    
    
    int p=0;
    for(size_t i=1;i<cloud_output->points.size();i++)
    {
	if(cloud_output->points[i].z > cloud_output->points[p].z && cloud_output->points[i].y < cloud_output->points[p].y)
	{
	    p=i;
	}
    }
    */
	//标记可视化
	/*
    visualization_msgs::Marker marker_1;
    if(p<cloud_output->points.size())
    {
	build_marker(marker_1,cloud_output->points[p].x,cloud_output->points[p].y,cloud_output->points[p].z);
	std::cout<<cloud_output->points[p].x<<cloud_output->points[p].y<<cloud_output->points[p].z<<std::endl;
    }
    else
    {
	build_marker(marker_1,1,1,1);
    }*/
	
    //cloud_output->points.resize( cloud_output->width * cloud_output->height );

