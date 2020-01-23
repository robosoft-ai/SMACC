#include <smacc/smacc.h>
namespace sm_dance_bot
{
struct StNavigateForward1 : smacc::SmaccState<StNavigateForward1, MsDanceBotRunMode>
{
  typedef mpl::list<
      // Expected event
      Transition<EvActionSucceeded<ClMoveBaseZ, OrNavigation>, StRotateDegrees2>,

      // Error events
      //Transition<smacc::EvTopicMessageTimeout<CbLidarSensor>, StAcquireSensors>,
      Transition<EvActionAborted<ClMoveBaseZ, OrNavigation>, StNavigateToWaypointsX, ABORT>,
      Transition<EvActionPreempted<ClMoveBaseZ, OrNavigation>, StNavigateToWaypointsX, PREEMPT>>
      reactions;

  using SmaccState::SmaccState;

  static void staticConfigure()
  {
    configure_orthogonal<OrNavigation, CbNavigateForward>(1);
    configure_orthogonal<OrLED, CbLEDOff>();
    configure_orthogonal<OrObstaclePerception, CbLidarSensor>();
  }

  void runtimeConfiguration()
  {
    ClMoveBaseZ *move_base_action_client;
    this->requiresClient(move_base_action_client);

    // we careful with the lifetime of the callbac, us a scoped connection if is not forever
    move_base_action_client->onSucceeded(&StNavigateForward1::onActionClientSucceeded, this);
  }

  void onActionClientSucceeded(ClMoveBaseZ::ResultConstPtr &msg)
  {
    ROS_INFO_STREAM(" [Callback SmaccSignal] Success Detected from StAquireSensors (connected to client signal), result data: " << *msg);
  }
};
} // namespace sm_dance_bot