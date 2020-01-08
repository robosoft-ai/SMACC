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
      smacc::Transition<EvWaypoint0<ClMoveBaseZ, OrNavigation>, SS1::SsRadialPattern1, TRANSITION_1>,
      smacc::Transition<EvWaypoint1<ClMoveBaseZ, OrNavigation>, SS1::SsRadialPattern1, TRANSITION_2>,
      smacc::Transition<EvWaypoint2<ClMoveBaseZ, OrNavigation>, SS1::SsRadialPattern1, TRANSITION_3>,

      // Error events
      //smacc::Transition<smacc::EvTopicMessageTimeout<CbLidarSensor>, StAcquireSensors>,
      smacc::Transition<EvActionAborted<ClMoveBaseZ, OrNavigation>, StNavigateToWaypointsX>>
      reactions;

  //int currentIteration;

  static void onDefinition()
  {
    //static_configure<OrObstaclePerception, CbLidarSensor>();
  }

  void onInitialize()
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