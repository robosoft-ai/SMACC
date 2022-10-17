#include <ros/ros.h>
#include <std_msgs/UInt16.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "numbers_publisher");

  ros::NodeHandle nh_;

  auto pub = nh_.advertise<std_msgs::UInt16>("/numbers", 1);

  ros::Rate r(0.5);

  int i = 0;
  while (ros::ok())
  {
    std_msgs::UInt16 msg;
    msg.data = i++;

    pub.publish(msg);

    r.sleep();
    ros::spinOnce();
  }
}
