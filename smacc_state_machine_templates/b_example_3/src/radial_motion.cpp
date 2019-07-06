#include <radial_motion.h>
#include <states/navigate_to_end_point.h>
#include <states/navigate_to_radial_start.h>
#include <states/return_to_radial_start.h>
#include <states/rotate_degrees.h>

#include <substates/navigate_to_end_point/navigate.h>
#include <substates/navigate_to_end_point/tool.h>

#include <substates/navigate_to_radial_start/navigate.h>
#include <substates/navigate_to_radial_start/tool.h>

#include <substates/return_to_radial_start/navigate.h>
#include <substates/return_to_radial_start/tool.h>

#include <substates/rotate_degrees/navigate.h>
#include <substates/rotate_degrees/tool.h>

#include <boost/thread.hpp>

//------------------------------------------------------------------------------

int main(int argc, char **argv) 
{
  ros::init(argc, argv, "radial_test_state_machine");
  ros::NodeHandle nh;

  smacc::run<RadialMotionStateMachine>();
}