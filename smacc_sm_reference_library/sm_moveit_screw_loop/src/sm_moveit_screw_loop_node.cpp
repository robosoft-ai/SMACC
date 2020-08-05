#include <sm_moveit_screw_loop/sm_moveit_screw_loop.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "sm_moveit_screw_loop");
  ros::NodeHandle nh;
  ROS_INFO("SM MOVEIT NODE STARTS");
  
  smacc::run<sm_moveit_screw_loop::SmFetchSixTablePickNSort1>();
  
}
