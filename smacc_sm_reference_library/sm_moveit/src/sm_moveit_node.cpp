#include <sm_moveit/sm_moveit.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "sm_moveit");
  ros::NodeHandle nh;
  ROS_INFO("SM MOVEIT NODE STARTS");
  
  smacc::run<sm_moveit::SmMoveIt>();
  
}
