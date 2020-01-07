#include <smacc/smacc.h>
namespace sm_dance_bot
{
struct StRotateDegrees4 : smacc::SmaccState<StRotateDegrees4, MsDanceBotRunMode>
{
  using SmaccState::SmaccState;

  typedef mpl::list<
      // Expected event
      smacc::Transition<EvActionSucceeded<ClMoveBaseZ, OrNavigation>, StNavigateReverse2>,

      // Error events
      //smacc::Transition<smacc::EvTopicMessageTimeout<CbLidarSensor>, StAcquireSensors>,
      smacc::Transition<EvActionAborted<ClMoveBaseZ, OrNavigation>, StNavigateToWaypointsX>>
      reactions;

  static void onDefinition()
  {
    static_configure<OrNavigation, CbRotate>(/*30*/ -180);
    static_configure<OrLED, CbLEDOff>();
    static_configure<OrObstaclePerception, CbLidarSensor>();
  }

  void onInitialize()
  {
  }
};
}