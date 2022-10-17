#include <ros/ros.h>
#include <std_msgs/Float32.h>

int main(int argc, char** argv)
{
    ros::init(argc,argv, "temperature_sensor_node");

    ros::NodeHandle nh;
    auto pub = nh.advertise<std_msgs::Float32>("/temperature",1);

    ros::Rate r(30);

    int i=0;
    while(ros::ok())
    {
        std_msgs::Float32 msg;
        // temperatures varies sinoidally from -20 to 20 degrees, but sporadically it has 32 deg
        if (i %100 ==0 )
            msg.data = 32;
        else
            msg.data = 20 * sin(0.1 * ros::Time::now().toSec());

        pub.publish(msg);

        r.sleep();
        ros::spinOnce();
        i++;
    }
}
