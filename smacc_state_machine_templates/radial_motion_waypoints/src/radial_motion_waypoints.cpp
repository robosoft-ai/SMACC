#include <radial_motion_waypoints/radial_motion_waypoints.h>

int main(int argc, char **argv) 
{
  ros::init(argc, argv, "radial_motion_waypoints");
  ros::NodeHandle nh;

  ros::Duration(5).sleep();
  smacc::run<RadialMotionWaypointsStateMachine>();
}