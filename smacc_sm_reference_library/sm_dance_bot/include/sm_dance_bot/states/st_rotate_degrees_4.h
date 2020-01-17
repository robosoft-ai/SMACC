#include <smacc/smacc.h>
namespace sm_dance_bot
{
struct StRotateDegrees4 : smacc::SmaccState<StRotateDegrees4, MsDanceBotRunMode>
{
  using SmaccState::SmaccState;

  typedef mpl::list<
      // Expected event
      Transition<EvActionSucceeded<ClMoveBaseZ, OrNavigation>, StNavigateReverse2>,

      // Error events
      //Transition<smacc::EvTopicMessageTimeout<CbLidarSensor>, StAcquireSensors>,
      Transition<EvActionAborted<ClMoveBaseZ, OrNavigation>, StNavigateToWaypointsX>>
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