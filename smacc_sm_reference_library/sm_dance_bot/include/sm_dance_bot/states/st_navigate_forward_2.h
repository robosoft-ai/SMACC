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

  static void staticConfigure()
  {
    configure_orthogonal<OrNavigation, CbNavigateForward>(1);
    configure_orthogonal<OrLED, CbLEDOff>();
    configure_orthogonal<OrObstaclePerception, CbLidarSensor>();
  }

  // Key N -> next state
  void runtimeConfigure()
  {
  }
};
} // namespace sm_dance_bot