#include <smacc/smacc.h>
namespace sm_dance_bot
{
struct StRotateDegrees5 : smacc::SmaccState<StRotateDegrees5, MsDanceBotRunMode>
{
  using SmaccState::SmaccState;

  typedef mpl::list<
      // Expected event
      smacc::transition<EvActionSucceeded<smacc::SmaccMoveBaseActionClient>, StNavigateToWaypointsX>,

      // Error events
      //smacc::transition<smacc::EvTopicMessageTimeout<SbLidarSensor>, StAcquireSensors>,
      smacc::transition<EvActionAborted<smacc::SmaccMoveBaseActionClient>, StNavigateToWaypointsX>>
      reactions;

  static void onDefinition()
  {
    static_configure<NavigationOrthogonal, SbRotate>(/*30*/ -180);
    static_configure<ToolOrthogonal, SbToolStop>();
    static_configure<ObstaclePerceptionOrthogonal, SbLidarSensor>();
  }

  void onInitialize()
  {
  }
};
}