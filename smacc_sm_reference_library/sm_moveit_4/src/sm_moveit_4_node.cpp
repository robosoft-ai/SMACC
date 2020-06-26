#include <sm_moveit_4/sm_moveit_4.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "sm_moveit_4");
  ros::NodeHandle nh;
  ROS_INFO("SM MOVEIT NODE STARTS");
  
  smacc::run<sm_moveit_4::SmMoveIt44>();
  
}
