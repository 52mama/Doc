#include <ros/ros.h> 

#include <pcl/PCLPointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/PointCloud2.h>

#include <visualization_msgs/Marker.h>

#include <pcl/keypoints/harris_3d.h>

#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>  

#include "opencv2/highgui/highgui.hpp"//画点
#include "opencv2/imgproc/imgproc.hpp"

#include "cv.h"    //绘制文字
#include "highgui.h" 

#include <string>//int转string
#include <iostream>

#include <vector>


ros::NodeHandle nh; 
ros::Subscriber sub; 
ros::Publisher pub;
std::vector<cv::Point> flag_point;
int x_mouse,y_mouse;
    
void calibration(); 
void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& input);
void gradient_descent();
void pcl_lidar(const sensor_msgs::PointCloud2ConstPtr& input);
void cv_onMouseHandle(int event,int x,int y,int flags, void *param);
void cv_camera();



void calibration()
{
    sub = nh.subscribe("input", 10, cloud_cb); 
    pub = nh.advertise<sensor_msgs::PointCloud2> ("process_output", 1);
}

void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& input) 
{
	 cv_camera();
	 pcl_lidar(input);
    }
void gradient_descent()
{
}
void pcl_lidar(const sensor_msgs::PointCloud2ConstPtr& input)
{
}
void cv_onMouseHandle(int event,int x,int y,int flags, void *param)
{
    //cv::Mat* image = (cv::Mat*) param;
    switch (event)
    {
    case CV_EVENT_MOUSEMOVE:
			    x_mouse=x;
			    y_mouse=y;
			    break;
    case CV_EVENT_LBUTTONDOWN:
			    flag_point.push_back(cv::Point(x,y));
			    break;
    }
}
void cv_camera()
{
    cv::VideoCapture cap;
    cap.open(1); //打开摄像头
    if(!cap.isOpened())	return ;
    cv::Mat image;
    
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
	for(int i=0;i<flag_point.size();i++)
	{
	    circle(image, flag_point[i] , 3 ,cv::Scalar(255,0,0),-1); 
	}
	
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
        if(cv::waitKey(1)!=255) 	break;	
    }
    cap.release();
    cv::destroyAllWindows();
}
