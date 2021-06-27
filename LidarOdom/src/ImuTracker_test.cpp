/**
 * 
 */
#include<ros/ros.h>
#include<std_msgs/String.h>

#include<sstream>


int main(int argc, char **argv)
{
    ros::init(argc, argv, "ImuTracker_test");
    ros::NodeHandle nh;
    ros::Rate loop_rate(1);

    ros::Publisher pub = nh.advertise<std_msgs::String>("num", 100);
    std::size_t num_ = 0;
    while (ros::ok())
    {
        std_msgs::String msg_;
        num_++;
        std::stringstream str_strm;
        str_strm << num_;
        msg_.data = str_strm.str();
        pub.publish(msg_);
        ROS_INFO("node_a send: [%s]", msg_.data.c_str());

        ros::spinOnce();

        loop_rate.sleep();
    }

    return 0;
}