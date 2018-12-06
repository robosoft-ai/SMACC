#include <waypoints_machine.h>
#include <states/go_to_odd_waypoint.h>
#include <states/go_to_even_waypoint.h>

#include <boost/thread.hpp>

//------------------------------------------------------------------------------

int main(int argc, char **argv) 
{
  ros::init(argc, argv, "waypoints_state_machine");
  ros::NodeHandle nh;

  smacc::run<WayPointsStateMachine>();
}