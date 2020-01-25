#include <smacc/smacc.h>
namespace sm_dance_bot
{
struct StNavigateToWaypoint1 : smacc::SmaccState<StNavigateToWaypoint1, MsDanceBotRunMode>
{
  using SmaccState::SmaccState;

  typedef mpl::list<
      // Expected event
      Transition<EvActionSucceeded<ClMoveBaseZ, OrNavigation>, StNavigateToWaypointsX>,

      // Error events
      //smacc::Transition<smacc::EvTopicMessageTimeout<CbLidarSensor>, StAcquireSensors>,
      Transition<EvActionAborted<ClMoveBaseZ, OrNavigation>, StNavigateToWaypointsX>>
      reactions;

  static void staticConfigure()
  {
    configure_orthogonal<OrNavigation, CbNavigateGlobalPosition>(0, 0, 0);
    configure_orthogonal<OrLED, CbLEDOn>();
    configure_orthogonal<OrStringPublisher, CbStringPublisher>("All Done!");
    configure_orthogonal<OrObstaclePerception, CbLidarSensor>();
  }

  void runtimeConfigure()
  {
    
  }
};
}