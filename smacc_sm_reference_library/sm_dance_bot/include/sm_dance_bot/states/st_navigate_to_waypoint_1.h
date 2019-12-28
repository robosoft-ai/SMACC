#include <smacc/smacc.h>
namespace sm_dance_bot
{
struct StNavigateToWaypoint1 : smacc::SmaccState<StNavigateToWaypoint1, MsDanceBotRunMode>
{
  using SmaccState::SmaccState;

  typedef mpl::list<
      // Expected event
      smacc::transition<EvActionSucceeded<ClMoveBaseZ, OrNavigation>, StNavigateToWaypointsX>,

      // Error events
      //smacc::transition<smacc::EvTopicMessageTimeout<CbLidarSensor>, StAcquireSensors>,
      smacc::transition<EvActionAborted<ClMoveBaseZ, OrNavigation>, StNavigateToWaypointsX>>
      reactions;

  static void onDefinition()
  {
    static_configure<OrNavigation, CbNavigateGlobalPosition>(0, 0, 0);
    static_configure<OrTool, CbToolStart>();
    static_configure<OrStringPublisher, CbStringPublisher>("All Done!");
    static_configure<OrObstaclePerception, CbLidarSensor>();
  }

  void onInitialize()
  {
    
  }
};
}