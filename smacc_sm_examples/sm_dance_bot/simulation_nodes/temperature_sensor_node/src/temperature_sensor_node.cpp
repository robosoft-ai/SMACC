#include <ros/ros.h>
#include <sensor_msgs/Temperature.h>

int main(int argc, char** argv)
{
    ros::init(argc,argv, "temperature_sensor_node");

    ros::NodeHandle nh;
    auto pub = nh.advertise<sensor_msgs::Temperature>("/temperature",1);

    ros::Rate r(10);

    while(ros::ok())
    {
        sensor_msgs::Temperature msg;
        pub.publish(msg);

        r.sleep();
        ros::spinOnce();
    }
}