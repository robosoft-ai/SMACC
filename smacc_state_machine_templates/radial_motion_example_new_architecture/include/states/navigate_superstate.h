#include <smacc/smacc.h>
class RadialMotionStateMachine;
class NavigateToRadialStart;

struct Superstate: smacc::SmaccState<Superstate,RadialMotionStateMachine ,  NavigateToRadialStart>
{
  using SmaccState::SmaccState;
};
