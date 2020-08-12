#include <sm_moveit_wine_serve/sm_moveit_wine_serve.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "sm_moveit_wine_serve");
  ros::NodeHandle nh;
  ROS_INFO("SM MOVEIT NODE STARTS");
  
  smacc::run<sm_moveit_wine_serve::SmFetchSixTablePickNSort1>();
  
}
