#include <smacc/smacc.h>
namespace sm_dance_bot_3
{
struct StRotateDegrees3 : smacc::SmaccState<StRotateDegrees3, MsDanceBotRunMode>
{
  using SmaccState::SmaccState;

  typedef mpl::list<
      // Expected event
      Transition<EvActionSucceeded<ClMoveBaseZ, OrNavigation>, StNavigateToWaypointsX>,

      // Error events
      //Transition<smacc::EvTopicMessageTimeout<CbLidarSensor>, StAcquireSensors>,
      Transition<EvActionAborted<ClMoveBaseZ, OrNavigation>, StNavigateToWaypointsX>>
      reactions;

  static void staticConfigure()
  {
    configure_orthogonal<OrNavigation, CbRotate>(/*30*/ 180);
    configure_orthogonal<OrLED, CbLEDOff>();
    configure_orthogonal<OrObstaclePerception, CbLidarSensor>();
  }

  void runtimeConfiguration()
  {
  }
};
}