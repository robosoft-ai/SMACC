#include <smacc/smacc.h>
namespace sm_dance_bot
{

//-------------------------------------------------
struct StNavigateToWaypointsX : smacc::SmaccState<StNavigateToWaypointsX, MsDanceBotRunMode>
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

      smacc::transition<EvWaypoint0<ClMoveBaseZ, OrNavigation>, sm_dance_bot::SS1::SsRadialPattern1, TRANSITION_1>,
      smacc::transition<EvWaypoint1<ClMoveBaseZ, OrNavigation>, sm_dance_bot::SS2::SsRadialPattern2, TRANSITION_2>,
      smacc::transition<EvWaypoint2<ClMoveBaseZ, OrNavigation>, sm_dance_bot::SS3::SsRadialPattern3, TRANSITION_3>,
      smacc::transition<EvWaypoint3<ClMoveBaseZ, OrNavigation>, sm_dance_bot::SS4::SsFPattern1, TRANSITION_4>,
      smacc::transition<EvWaypoint4<ClMoveBaseZ, OrNavigation>, sm_dance_bot::SS5::SsSPattern1, TRANSITION_5>,

      // Error events
      //smacc::transition<smacc::EvTopicMessageTimeout<CbLidarSensor>, StAcquireSensors>,
      smacc::transition<EvActionAborted<smacc::ClMoveBaseZ, OrNavigation>, StNavigateToWaypointsX>>
      reactions;

  //int currentIteration;

  static void onDefinition()
  {
    static_configure<OrTool, CbToolStart>();
    static_configure<OrObstaclePerception, CbLidarSensor>();
  }

  void onInitialize()
  {
    ClMoveBaseZ *move_base;
    this->requiresClient(move_base);

    auto waypointsNavigator = move_base->getComponent<WaypointNavigator>();
    loadWaypointsFirstTime(waypointsNavigator);

    waypointsNavigator->sendNextGoal();
    ROS_INFO("current iteration waypoints x: %ld", waypointsNavigator->getCurrentWaypointIndex());
  }

  void loadWaypointsFirstTime(WaypointNavigator *waypointsNavigator)
  {
    // if it is the first time and the waypoints navigator is not configured
    if (waypointsNavigator->getWaypoints().size() == 0)
    {
      std::string planfilepath;
      ros::NodeHandle nh("~");
      if (nh.getParam("/sm_dance_bot/waypoints_plan", planfilepath))
      {
        waypointsNavigator->loadWayPointsFromFile(planfilepath);
      }
    }
  }
};

} // namespace sm_dance_bot