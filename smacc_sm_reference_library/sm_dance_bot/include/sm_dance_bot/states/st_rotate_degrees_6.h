#include <smacc/smacc.h>
namespace sm_dance_bot
{
struct StRotateDegrees6 : smacc::SmaccState<StRotateDegrees6, MsDanceBotRunMode>
{
  using SmaccState::SmaccState;

  typedef mpl::list<
      // Expected event
      smacc::transition<EvActionSucceeded<smacc::SmaccMoveBaseActionClient>, StNavigateReverse3>,

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