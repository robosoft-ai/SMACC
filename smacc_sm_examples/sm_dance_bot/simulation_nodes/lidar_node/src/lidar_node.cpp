#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

int main(int argc, char** argv)
{
    ros::init(argc,argv, "lidar_node");


    ros::spin();

}