#include <radial_motion.h>

#include <states/navigate_to_radial_start.h>
#include <states/navigate_to_end_point.h>
#include <states/return_to_radial_start.h>
#include <states/rotate_degrees.h>

#include <boost/thread.hpp>

//------------------------------------------------------------------------------
void RadialMotionStateMachine::mapSmaccStateBehaviors()
{
    this->mapBehavior<ToolStop>(NavigateToRadialStart::ToolBehaviorKeyName);
    this->mapBehavior<ToolStart>(NavigateToEndPoint::ToolBehaviorKeyName);
    this->mapBehavior<ToolStop>(NavigateToEndPoint::ToolBehaviorKeyName);
    this->mapBehavior<ToolStop>(RotateDegress::ToolBehaviorKeyName);
}

int main(int argc, char **argv) 
{
  ros::init(argc, argv, "radial_test_state_machine");
  smacc::run<RadialMotionStateMachine>();
}

