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

    // example for temperature sensor

    //std::function<bool(EvTopicMessage<CbConditionClTemperatureSensor> *)> temperatureWarningCondition = ([](EvTopicMessage<CbConditionClTemperatureSensor> *ev) {
    //  return ev->msgData.temperature > 30;
    //});
    //static_createLogicUnit<LuConditional, EvTrue<LuConditional>, mpl::list<EvTopicMessage<CbConditionClTemperatureSensor>>>(temperatureWarningCondition);

    //smacc::transition<EvTrue<LuConditional>, StNavigateToWaypointsX>>

    /*
    std::vector<Pose2D> waypoints =
        {
            {3.0, -2.0, 0},
            {-5.5, 3.0, 0},
            {-11.0, -5.0, 0},
            {2.0, -8.58, 0},
            {-10.0, 14.5, 0}};
*/
    //static_configure<OrNavigation, CbWaypointsNavigator>(waypoints);

    /*
    static_createLogicUnit<LuRoundRobin, 
                              mpl::list<
                                   EvWaypoint1<StNavigateToWaypointsX>,
                                   EvWaypoint2<StNavigateToWaypointsX>,
                                   EvWaypoint3<StNavigateToWaypointsX>, 
                                   EvWaypoint4<StNavigateToWaypointsX>,
                                   EvWaypoint5<StNavigateToWaypointsX>
                                  >, 
                         mpl::list<EvActionSucceeded<smacc::ClMoveBaseZ, OrNavigation>>();
                         */

    // what about persistence
    // would we have any output event?

    // WaypointNaigator
    // it is a logic unit
    // it does not contain the transition table internally

    // static_transition_dynamic_logic_unit< sm_dance_bot::SS1::SsRadialPattern1,
    //                                       sm_dance_bot::SS2::SsRadialPattern2,
    //                                       sm_dance_bot::SS2::SsRadialPattern3>();

    // static_createLogicUnit<lusource, outevent, invent, dststate1, dststate2 ...>();
    // static_createLogicUnit<LuCounter, EvCountFinish<LuCounter>,    >();
    // function<T1, T2, Targs...>
    // function<T1, T2, TOutEvents..., TInEvents...>

    // function<LogicUnitType, mpl::list<>, mpl::list<>>();
    // function<LogicUnitType, TEvOut, mpl::list<>>();
    // function<LogicUnitType, mpl::list<>, TEvIn>();

    // static_transition(
    // (
    //   {
    //     {0, typeid(SS1::SsRadialPattern1)},
    //     {1, typeid(SS1::SsRadialPattern2)}
    //     {3, typeid(SS1::SsRadialPattern3)}
    //   }
    // ));
  }

  void onInitialize()
  {
    // this->currentIteration = 0;

    // // IDEA: this->declarePersistentVariable(&currentIteration)
    // this->getGlobalSMData("navigation_x_iteration", currentIteration);

    //ROS_INFO("current iteration waypoints x: %d", currentIteration);

    ClMoveBaseZ* move_base;
    this->requiresClient(move_base);

    auto waypointsNavigator = move_base->getComponent<WaypointNavigator>();

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

    //move_base->getComponent<WaypointNavigator>().sendNextGoal();
    waypointsNavigator->sendNextGoal();
    ROS_INFO("current iteration waypoints x: %ld", waypointsNavigator->getCurrentWaypointIndex());
  }
};

} // namespace sm_dance_bot