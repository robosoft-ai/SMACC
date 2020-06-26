#include <smacc/smacc.h>
namespace sm_dance_bot_2
{
// STATE DECLARATION
struct StRotateDegrees4 : smacc::SmaccState<StRotateDegrees4, MsDanceBotRunMode>
{
  using SmaccState::SmaccState;

// TRANSITION TABLE
  typedef mpl::list<

  Transition<EvActionSucceeded<ClMoveBaseZ, OrNavigation>, StNavigateReverse2>,
  Transition<EvActionAborted<ClMoveBaseZ, OrNavigation>, StNavigateToWaypointsX>
  
  >reactions;

// STATE FUNCTIONS
  static void staticConfigure()
  {
    configure_orthogonal<OrNavigation, CbRotate>(/*30*/ -180);
    configure_orthogonal<OrLED, CbLEDOff>();
    configure_orthogonal<OrObstaclePerception, CbLidarSensor>();
  }
};
}