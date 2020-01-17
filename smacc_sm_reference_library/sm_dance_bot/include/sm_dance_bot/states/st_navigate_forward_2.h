#include <smacc/smacc.h>
namespace sm_dance_bot
{
struct StNavigateForward2 : smacc::SmaccState<StNavigateForward2, MsDanceBotRunMode>
{
  typedef mpl::list<
      // Expected event
      Transition<EvActionSucceeded<ClMoveBaseZ, OrNavigation>, StRotateDegrees5>,

      // Error events
      //Transition<smacc::EvTopicMessageTimeout<CbLidarSensor>, StAcquireSensors>,
      Transition<EvActionAborted<ClMoveBaseZ, OrNavigation>, StNavigateToWaypointsX>>
      reactions;

  using SmaccState::SmaccState;

  static void onDefinition()
  {
    static_configure<OrNavigation, CbNavigateForward>(1);
    static_configure<OrLED, CbLEDOff>();
    static_configure<OrObstaclePerception, CbLidarSensor>();
  }

  // Key N -> next state
  void onInitialize()
  {
  }
};
} // namespace sm_dance_bot