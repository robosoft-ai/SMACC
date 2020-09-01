#include <smacc/smacc.h>
namespace sm_ridgeback_floor_coverage_static_1
{
  // STATE DECLARATION
  struct StNavigateForward1 : smacc::SmaccState<StNavigateForward1, MsDanceBotRunMode>
  {
    using SmaccState::SmaccState;

    // TRANSITION TABLE
    typedef mpl::list<

        Transition<EvCbSuccess<CbNavigateForward, OrNavigation>, StRotateDegrees2>,
        Transition<EvCbFailure<CbNavigateForward, OrNavigation>, StNavigateToWaypointsX, ABORT>,
        Transition<EvActionPreempted<CbNavigateForward, OrNavigation>, StNavigateToWaypointsX, PREEMPT>

        >
        reactions;

    // STATE FUNCTIONS
    static void staticConfigure()
    {
      configure_orthogonal<OrNavigation, CbNavigateForward>(1);
      configure_orthogonal<OrLED, CbLEDOff>();
      configure_orthogonal<OrObstaclePerception, CbLidarSensor>();
    }

    void runtimeConfigure()
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
} // namespace sm_ridgeback_floor_coverage_static_1