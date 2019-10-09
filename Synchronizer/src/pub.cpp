#include <ros/ros.h>                         //类似 C 语言的 stdio.h 
#include <std_msgs/Float32.h> //要用到 msg 中定义的数据类型
 
int main(int argc,char **argv)
{    
   ros::init(argc,argv,"talker");            //解析参数，命名节点为 talker
   ros::NodeHandle nh;                       //创建句柄，相当于一套工具，可以实例化 node，并且对 node 进行操作
   

   std_msgs::Float32 msg ;
   msg.data = 11;
   
   ros::Publisher pub = nh.advertise<std_msgs::Float32>("/pub_test_a",1);//创建 publisher 对象
   ros::Rate loop_rate(1.0);                 //创建 rate 对象，定义循环发布的频率，1 HZ
   while(ros::ok())
   {  
      
      
      pub.publish(msg);                      //发布消息
      loop_rate.sleep();                     //根据定义的发布频率，sleep
   }
   return 0;
}