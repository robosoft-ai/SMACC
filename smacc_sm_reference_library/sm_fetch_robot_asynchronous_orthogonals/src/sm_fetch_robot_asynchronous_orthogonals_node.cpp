#include <sm_fetch_robot_asynchronous_orthogonals/sm_fetch_robot_asynchronous_orthogonals.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "sm_fetch_robot_asynchronous_orthogonals");
  ros::NodeHandle nh;
  ROS_INFO("SM MOVEIT NODE STARTS");
  
  smacc::run<sm_fetch_robot_asynchronous_orthogonals::SmFetchRobotAsynchronousOrthogonals>(); 
}