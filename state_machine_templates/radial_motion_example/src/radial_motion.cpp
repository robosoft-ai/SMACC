#include <radial_motion.h>
#include <states/navigate_to_end_point.h>
#include <states/navigate_to_radial_start.h>
#include <states/return_to_radial_start.h>
#include <states/rotate_degrees.h>
#include <boost/thread.hpp>

//------------------------------------------------------------------------------

int main(int argc, char **argv) 
{
  ros::init(argc, argv, "radial_test_state_machine");
  ros::NodeHandle nh;

  smacc::run<RadialMotionStateMachine>();
}