#include <sm_moveit_3/sm_moveit_3.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "sm_moveit_3");
  ros::NodeHandle nh;
  ROS_INFO("SM MOVEIT NODE STARTS");
  
  smacc::run<sm_moveit_3::SmMoveit3>();
  
}
