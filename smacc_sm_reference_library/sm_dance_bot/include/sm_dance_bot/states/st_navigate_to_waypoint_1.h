#include <smacc/smacc.h>
namespace sm_dance_bot
{
struct StNavigateToWaypoint1 : smacc::SmaccState<StNavigateToWaypoint1, MsDanceBotRunMode>
{
  using SmaccState::SmaccState;

  typedef mpl::list<
      // Expected event
      smacc::Transition<EvActionSucceeded<ClMoveBaseZ, OrNavigation>, StNavigateToWaypointsX>,

      // Error events
      //smacc::Transition<smacc::EvTopicMessageTimeout<CbLidarSensor>, StAcquireSensors>,
      smacc::Transition<EvActionAborted<ClMoveBaseZ, OrNavigation>, StNavigateToWaypointsX>>
      reactions;

  static void onDefinition()
  {
    static_configure<OrNavigation, CbNavigateGlobalPosition>(0, 0, 0);
    static_configure<OrLED, CbLEDOn>();
    static_configure<OrStringPublisher, CbStringPublisher>("All Done!");
    static_configure<OrObstaclePerception, CbLidarSensor>();
  }

  void onInitialize()
  {
    
  }
};
}