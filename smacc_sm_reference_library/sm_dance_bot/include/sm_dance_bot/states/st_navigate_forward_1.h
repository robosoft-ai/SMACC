#include <smacc/smacc.h>
namespace sm_dance_bot
{
struct StNavigateForward1 : smacc::SmaccState<StNavigateForward1, MsDanceBotRunMode>
{
  typedef mpl::list<
      // Expected event
      transition<EvActionSucceeded<SmaccMoveBaseActionClient, NavigationOrthogonal>, StRotateDegrees2>,

      // Error events
      //smacc::transition<smacc::EvTopicMessageTimeout<SbLidarSensor>, StAcquireSensors>,
      smacc::transition<EvActionAborted<smacc::SmaccMoveBaseActionClient, NavigationOrthogonal>, StNavigateToWaypointsX, ABORT>,
      smacc::transition<EvActionPreempted<smacc::SmaccMoveBaseActionClient, NavigationOrthogonal>, StNavigateToWaypointsX, PREEMPT>>
      reactions;

  using SmaccState::SmaccState;

  static void onDefinition()
  {
    static_configure<NavigationOrthogonal, SbNavigateForward>(1);
    static_configure<ToolOrthogonal, SbToolStop>();
    static_configure<ObstaclePerceptionOrthogonal, SbLidarSensor>();
  }

  void onInitialize()
  {
    SmaccMoveBaseActionClient *move_base_action_client;
    this->requiresClient(move_base_action_client);

    // we careful with the lifetime of the callbac, us a scoped connection if is not forever
    move_base_action_client->onSucceeded(&StNavigateForward1::onActionClientSucceded, this);
  }

  void onActionClientSucceded(SmaccMoveBaseActionClient::ResultConstPtr &msg)
  {
    ROS_INFO_STREAM(" [Callback SmaccSignal] Success Detected from StAquireSensors (connected to client signal), result data: " << *msg);
  }
};
} // namespace sm_dance_bot