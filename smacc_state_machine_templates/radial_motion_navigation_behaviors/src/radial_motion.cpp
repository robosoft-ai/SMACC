#include <radial_motion.h>

// state behaviors
#include <substates_behaviors/tool/tool_start.h>
#include <substates_behaviors/tool/tool_stop.h>

#include <substates_behaviors/navigation/navigate_global_position.h>
#include <substates_behaviors/navigation/navigate_forward.h>
#include <substates_behaviors/navigation/rotate.h>
#include <substates_behaviors/navigation/undo_path_backwards.h>

//------------------------------------------------------------------------------
void RadialMotionStateMachine::mapSmaccStateBehaviors()
{
    this->mapBehavior<NavigateToRadialStart::ToolSubstate, ToolStop>();
    this->mapBehavior<NavigateToEndPoint::ToolSubstate, ToolStart>();
    this->mapBehavior<ReturnToRadialStart::ToolSubstate, ToolStop>();
    this->mapBehavior<RotateDegress::ToolSubstate, ToolStop>();

    this->mapBehavior<NavigateToRadialStart::Navigate, NavigateGlobalPosition>();
    this->mapBehavior<NavigateToEndPoint::Navigate, NavigateForward>();
    this->mapBehavior<ReturnToRadialStart::Navigate, UndoPathBackwards>();
    this->mapBehavior<RotateDegress::Navigate, Rotate>();
}

int main(int argc, char **argv) 
{
  ros::init(argc, argv, "radial_test_state_machine");
  smacc::run<RadialMotionStateMachine>();
}