#include <sm_fetch_two_table_pick_n_place_1/sm_fetch_two_table_pick_n_place_1.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "sm_fetch_two_table_pick_n_place_1");
  ros::NodeHandle nh;
  ROS_INFO("SM MOVEIT NODE STARTS");

  smacc::run<sm_fetch_two_table_pick_n_place_1::SmFetchTwoTablePickNPlace1>();

}
