#include <smacc/smacc.h>
namespace sm_dance_bot
{
struct StNavigateToWaypoint1 : smacc::SmaccState<StNavigateToWaypoint1, MsDanceBotRunMode>
{
  using SmaccState::SmaccState;

  typedef mpl::list<
      // Expected event
      smacc::transition<EvActionSucceeded<smacc::SmaccMoveBaseActionClient, NavigationOrthogonal>, StNavigateToWaypointsX>,

      // Error events
      //smacc::transition<smacc::EvTopicMessageTimeout<CbLidarSensor>, StAcquireSensors>,
      smacc::transition<EvActionAborted<smacc::SmaccMoveBaseActionClient, NavigationOrthogonal>, StNavigateToWaypointsX>>
      reactions;

  static void onDefinition()
  {
    static_configure<NavigationOrthogonal, CbNavigateGlobalPosition>(0, 0, 0);
    static_configure<ToolOrthogonal, CbToolStart>();
    static_configure<PublisherOrthogonal, CbStringPublisher>("All Done!");
    static_configure<ObstaclePerceptionOrthogonal, CbLidarSensor>();
  }

  void onInitialize()
  {
    
  }
};
}