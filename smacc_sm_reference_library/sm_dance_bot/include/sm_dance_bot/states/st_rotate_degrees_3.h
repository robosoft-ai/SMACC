#include <smacc/smacc.h>
namespace sm_dance_bot
{
struct StRotateDegrees3 : smacc::SmaccState<StRotateDegrees3, MsDanceBotRunMode>
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
    static_configure<NavigationOrthogonal, CbRotate>(/*30*/ 180);
    static_configure<ToolOrthogonal, CbToolStop>();
    static_configure<ObstaclePerceptionOrthogonal, CbLidarSensor>();
  }

  void onInitialize()
  {
  }
};
}