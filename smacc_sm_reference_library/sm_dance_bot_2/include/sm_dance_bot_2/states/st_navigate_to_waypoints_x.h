#include <smacc/smacc.h>

namespace sm_dance_bot_2
{

using namespace move_base_z_client;

//-------------------------------------------------
struct StNavigateToWaypointsX : smacc::SmaccState<StNavigateToWaypointsX, SmDanceBot2>
{
  using SmaccState::SmaccState;

  struct TRANSITION_1 : SUCCESS
  {
  };
  struct TRANSITION_2 : SUCCESS
  {
  };
  struct TRANSITION_3 : SUCCESS
  {
  };
  struct TRANSITION_4 : SUCCESS
  {
  };
  struct TRANSITION_5 : SUCCESS
  {
  };

  typedef mpl::list<
      Transition<EvWaypoint0<ClMoveBaseZ, OrNavigation>, SS1::SsRadialPattern1, TRANSITION_1>,
      Transition<EvWaypoint1<ClMoveBaseZ, OrNavigation>, SS1::SsRadialPattern1, TRANSITION_2>,
      Transition<EvWaypoint2<ClMoveBaseZ, OrNavigation>, SS1::SsRadialPattern1, TRANSITION_3>,

      // Error events
      //Transition<smacc::EvTopicMessageTimeout<CbLidarSensor>, StAcquireSensors>,
      Transition<EvActionAborted<ClMoveBaseZ, OrNavigation>, StNavigateToWaypointsX>>
      reactions;

  //int currentIteration;

  static void staticConfigure()
  {
    //configure_orthogonal<OrObstaclePerception, CbLidarSensor>();
  }

  void runtimeConfigure()
  {
  }

  void onEntry()
  {
    ClMoveBaseZ *move_base;
    this->requiresClient(move_base);

    auto waypointsNavigator = move_base->getComponent<WaypointNavigator>();
    waypointsNavigator->sendNextGoal();
    ROS_INFO("current iteration waypoints x: %ld", waypointsNavigator->getCurrentWaypointIndex());
  }
};

} // namespace sm_dance_bot_2