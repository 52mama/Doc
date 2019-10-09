
#include <ros/ros.h>
#include <ros/console.h>

#include <vector>
#include <iostream>
#include <string>

#include <std_msgs/Float32.h>

template<class receive_type>
struct msg_Queue
{
private:

    struct time_and_msg
    {
        time_and_msg() : msg(nullptr) {;}
        time_and_msg(ros::Time time_ , receive_type* msg_  ) : time(time_) , msg(msg_)   {}
        ~time_and_msg() { delete msg; }

        ros::Time time;
        receive_type* msg;
    };

    std::vector<time_and_msg> queue;

    void call_back(receive_type in);

    receive_type* getMessage(ros::Time in);
    

    size_t begin;
    size_t end;
    size_t queue_size;

    std::string sub_topic;
    ros::Subscriber subscriber;


public:
    msg_Queue(size_t queue_size,std::string sub_topic,ros::NodeHandle &nh);
    ~msg_Queue();
};


template<class receive_type>
msg_Queue<receive_type>::msg_Queue(size_t queue_size_ , std::string sub_topic_, ros::NodeHandle &nh)
{
    //forbidden
    if(queue_size_ <= 2) { ; }
    
    sub_topic = sub_topic_;
    subscriber = nh.subscribe( sub_topic , 5, &msg_Queue<receive_type>::call_back ,this );
    queue.resize(queue_size);
    queue_size = queue_size_;
    begin = 0;
    end = begin;
}

template<class receive_type>
void msg_Queue<receive_type>::call_back(receive_type in)
{
    if( (end+1)%queue_size == begin )
    {
        queue[begin] = time_and_msg();
        begin = (begin+1)%queue_size;
        ROS_INFO("receive %s Queue is out of size",sub_topic);
    }
    queue[end].time = ros::Time::now();
    
    
    queue[end].msg = &in;
    end = (end+1)%queue_size;


    for(size_t i = 0; i < queue_size ; i++ ) 
    {
        if( i == begin )
        {
            std::cout<<"b ";    
        }
        else if( i == end )
        {
            std::cout<<"e ";
        }   
        else if( queue[i].msg == nullptr )
            std::cout<<"* ";
        else
            std::cout<<i<<" ";
        
    }   
    std::cout<<std::endl;
}

template<class receive_type>
msg_Queue<receive_type>::~msg_Queue()
{
    queue.clear();
}

template<class receive_type>
receive_type* msg_Queue<receive_type>::getMessage(ros::Time in)
{
    size_t min;
    ros::Time min_time = ros::Time::now();
    for(size_t i = begin ; i != end  ; i = (i+1)%queue_size )
    {
        if( ( in - queue[i].time ).toSec() > min_time.toSec() )
        {
            min = i;
            min_time =  in - queue[i].time ;
        }
    }
    begin = ( min + 1 ) % queue_size ;
    
    return queue[min].msg;
}

int main(int argc , char **argv )
{

    ros::init(argc,argv,"msg_queue");
    ros::NodeHandle nh;

    msg_Queue<std_msgs::Float32> a(5,"/pub_test_a",nh); 
   
    ros::spin();

    return 0;

}
