#include <smacc/smacc.h>
namespace sm_dance_bot
{
struct StRotateDegrees1 : smacc::SmaccState<StRotateDegrees1, MsDanceBotRunMode>
{
  using SmaccState::SmaccState;

  typedef mpl::list<
      // Expected event
      Transition<EvActionSucceeded<ClMoveBaseZ, OrNavigation>, StNavigateForward1>,

      // Error events
      //Transition<smacc::EvTopicMessageTimeout<CbLidarSensor>, StAcquireSensors>,
      Transition<EvActionAborted<ClMoveBaseZ, OrNavigation>, StNavigateToWaypointsX>>
      reactions;

  static void onDefinition()
  {
    static_configure<OrNavigation, CbRotate>(/*30*/ 90);
    static_configure<OrLED, CbLEDOff>();
    static_configure<OrObstaclePerception, CbLidarSensor>();
  }

  void onInitialize()
  {
    
  }
};
}