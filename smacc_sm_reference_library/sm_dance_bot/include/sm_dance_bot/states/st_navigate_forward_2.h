#include <smacc/smacc.h>
namespace sm_dance_bot
{
struct StNavigateForward2 : smacc::SmaccState<StNavigateForward2, MsDanceBotRunMode>
{
  typedef mpl::list<
      // Expected event
      smacc::transition<EvActionSucceeded<SmaccMoveBaseActionClient>, StRotateDegrees5>,

      // Error events
      //smacc::transition<smacc::EvTopicMessageTimeout<SbLidarSensor>, StAcquireSensors>,
      smacc::transition<EvActionAborted<smacc::SmaccMoveBaseActionClient>, StNavigateToWaypointsX>
      >
      reactions;

  using SmaccState::SmaccState;

  static void onDefinition()
  {
    static_configure<NavigationOrthogonal,SbNavigateForward>(1);
    static_configure<ToolOrthogonal,SbToolStop>();
    static_configure<ObstaclePerceptionOrthogonal, SbLidarSensor>();
  }

  // Key N -> next state
  void onInitialize()
  {
  }
};
}