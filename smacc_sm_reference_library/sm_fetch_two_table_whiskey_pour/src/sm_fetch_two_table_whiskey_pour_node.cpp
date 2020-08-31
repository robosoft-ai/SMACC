#include <sm_fetch_two_table_whiskey_pour/sm_fetch_two_table_whiskey_pour.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "sm_fetch_two_table_whiskey_pour");
  ros::NodeHandle nh;
  ROS_INFO("SM MOVEIT NODE STARTS");
  
  smacc::run<sm_fetch_two_table_whiskey_pour::SmFetchTwoTableWhiskeyPour>();
  
}
