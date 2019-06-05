#include <radial_motion.h>

#include <states/navigate_to_radial_start.h>
#include <states/navigate_to_end_point.h>
#include <states/return_to_radial_start.h>
#include <states/rotate_degrees.h>

#include <substates/navigation/navigate_to_radial_start.h>
#include <substates/navigation/navigate_to_end_point.h>
#include <substates/navigation/return_to_radial_start.h>
#include <substates/navigation/rotate_degrees.h>

#include <boost/thread.hpp>

//------------------------------------------------------------------------------
void RadialMotionStateMachine::mapSmaccStateBehaviors()
{
    this->mapBehavior<NavigateToRadialStart::ToolSubstate, ToolStop>();
    this->mapBehavior<NavigateToEndPoint::ToolSubstate, ToolStart>();
    this->mapBehavior<ReturnToRadialStart::ToolSubstate, ToolStop>();
    this->mapBehavior<RotateDegress::ToolSubstate, ToolStop>();
}

int main(int argc, char **argv) 
{
  ros::init(argc, argv, "radial_test_state_machine");
  smacc::run<RadialMotionStateMachine>();
}

