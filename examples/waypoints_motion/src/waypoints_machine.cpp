#include <waypoints_state.h>
#include <states/navigate_to_end_point.h>
#include <states/navigate_to_radial_start.h>

#include <boost/thread.hpp>

//------------------------------------------------------------------------------

int main(int argc, char **argv) 
{
  ros::init(argc, argv, "waypoints_state_machine");
  ros::NodeHandle nh;

  smacc::run<WaypointsStateMachine>();
}