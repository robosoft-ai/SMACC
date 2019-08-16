#include <smacc/smacc.h>
class RadialMotionStateMachine;
class NavigateToRadialStart;

struct Superstate: smacc::SmaccState<Superstate,RadialMotionStateMachineReplacement ,  NavigateToRadialStart>
{
  using SmaccState::SmaccState;
};
