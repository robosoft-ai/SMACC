#include <smacc/smacc.h>
namespace sm_dance_bot
{
struct StNavigateForward2 : smacc::SmaccState<StNavigateForward2, MsDanceBotRunMode>
{
  typedef mpl::list<
      // Expected event
      smacc::transition<EvActionSucceeded<SmaccMoveBaseActionClient, NavigationOrthogonal>, StRotateDegrees5>,

      // Error events
      //smacc::transition<smacc::EvTopicMessageTimeout<CbLidarSensor>, StAcquireSensors>,
      smacc::transition<EvActionAborted<smacc::SmaccMoveBaseActionClient, NavigationOrthogonal>, StNavigateToWaypointsX>>
      reactions;

  using SmaccState::SmaccState;

  static void onDefinition()
  {
    static_configure<NavigationOrthogonal, CbNavigateForward>(1);
    static_configure<ToolOrthogonal, CbToolStop>();
    static_configure<ObstaclePerceptionOrthogonal, CbLidarSensor>();
  }

  // Key N -> next state
  void onInitialize()
  {
  }
};
} // namespace sm_dance_bot