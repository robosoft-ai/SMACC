

template <typename TSource>
struct EvWaypoint1 : sc::event<EvWaypoint1<TSource>>
{
  
};

template <typename TSource>
struct EvWaypoint2 : sc::event<EvWaypoint2<TSource>>
{
  
};

template <typename TSource>
struct EvWaypoint3 : sc::event<EvWaypoint3<TSource>>
{
  
};

template <typename TSource>
struct EvWaypoint4 : sc::event<EvWaypoint4<TSource>>
{
  
};

template <typename TSource>
struct EvWaypoint5 : sc::event<EvWaypoint5<TSource>>
{
  
};

struct Pose2D
{
  Pose2D(double x, double y, double yaw)
  {
    this->x_ = x;
    this->y_ = y;
    this->yaw_ = yaw;
  }

  double x_;
  double y_;
  double yaw_;
};

//-------------------------------------------------
struct StNavigateToWaypointsX : smacc::SmaccState<StNavigateToWaypointsX, MsDanceBotRunMode>
{
  using SmaccState::SmaccState;

  struct TRANSITION_1: SUCCESS{};
  struct TRANSITION_2: SUCCESS{};
  struct TRANSITION_3: SUCCESS{};
  struct TRANSITION_4: SUCCESS{};
  struct TRANSITION_5: SUCCESS{};
  
  typedef mpl::list<
      // Expected event
      sc::custom_reaction<EvActionSucceded<smacc::SmaccMoveBaseActionClient>>,

      //smacc::transition<EvCounter<LuCounter, 14>, SS1::SsRadialPattern1>,
      //smacc::transition<EvCounter<LuCounter, 19>, SS1::SsRadialPattern1>,
      //smacc::transition<EvWaypoint1, SS1::SsRadialPattern1>,
      smacc::transition<EvWaypoint1<StNavigateToWaypointsX>, SS1::SsRadialPattern1, TRANSITION_1>,
      smacc::transition<EvWaypoint2<StNavigateToWaypointsX>, SS2::SsRadialPattern2, TRANSITION_2>,
      smacc::transition<EvWaypoint3<StNavigateToWaypointsX>, SS3::SsRadialPattern3, TRANSITION_3>,
      smacc::transition<EvWaypoint4<StNavigateToWaypointsX>, SS4::SsFPattern1, TRANSITION_4>,
      smacc::transition<EvWaypoint5<StNavigateToWaypointsX>, SS5::SsSPattern1, TRANSITION_5>,

      // Keyboard event
      sc::custom_reaction<EvKeyPressN<SbKeyboard>>,
      sc::custom_reaction<EvKeyPressP<SbKeyboard>>,

      // Error events
      //smacc::transition<smacc::EvTopicMessageTimeout<SbLidarSensor>, StAcquireSensors>,
      smacc::transition<EvActionAborted<smacc::SmaccMoveBaseActionClient>, StNavigateToWaypointsX>>
      reactions;

  int currentIteration;

  static void onDefinition()
  {
    static_configure<ToolOrthogonal, SbToolStart>();
    static_configure<KeyboardOrthogonal, SbKeyboard>();
    static_configure<ObstaclePerceptionOrthogonal, SbLidarSensor>();

    //static_createLogicUnit<LuAllEventsGo, EvAll<LuAllEventsGo, Unit1>, mpl::list<EvTopicMessage<SbLidarSensor>, EvTopicMessage<SbConditionTemperatureSensor>>>();

    //static_createLogicUnit<LuCounter, mpl::list<EvCounter1<LuCounter>, EvCounter2<LuCounter>, EvCounter1<LuCounter>, EvCounter4<LuCounter> EvCounter5<LuCounter>>,
    //                       EvActionSucceded<MoveBaseClient>>();

    //static_createLogicUnit<LuCounter, LuCounter::GenerateListEvents<20>::type, EvActionSucceded<MoveBaseClient>>();

    //static_createLogicUnit<LuAllEventsGo, EvAll<LuAllEventsGo, Unit1>,           EvTopicMessage<SbLidarSensor>, EvTopicMessage<SbConditionTemperatureSensor>>();

    // static_transition_dynamic_logic_unit< SS1::SsRadialPattern1,
    //                                       SS2::SsRadialPattern2,
    //                                       SS2::SsRadialPattern3>();

    // static_createLogicUnit<lusource, outevent, invent, dststate1, dststate2 ...>();
    // static_createLogicUnit<LuCounter, EvCountFinish<LuCounter>,    >();
    // function<T1, T2, Targs...>
    // function<T1, T2, TOutEvents..., TInEvents...>

    // function<LogicUnitType, mpl::list<>, mpl::list<>>();
    // function<LogicUnitType, TEvOut, mpl::list<>>();
    // function<LogicUnitType, mpl::list<>, TEvIn>();

    //static_createLogicUnit<LuAllEventsGo,  EvTopicMessage<SbLidarSensor>, EvTopicMessage<SbConditionTemperatureSensor>>();

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
    this->currentIteration = 0;

    // IDEA: this->declarePersistentVariable(&currentIteration)
    this->getGlobalSMData("navigation_x_iteration", currentIteration);

    ROS_INFO("current iteration waypoints x: %d", currentIteration);

    // x, y, yaw (orientation)
    std::vector<Pose2D> waypoints =
        {
            {3.0, -2.0, 0},
            {-5.5, 3.0, 0},
            {-11.0, -5.0, 0},
            {2.0, -8.58, 0},
            {-10.0, 14.5, 0}};

    if (currentIteration > waypoints.size())
    {
      ROS_FATAL("[WaypointsX] Out of range of waypoint list. Cannot go to waypoint %d.", this->currentIteration);
      this->throwFinishEvent();
    }
    else
    {
      auto &target = waypoints[currentIteration];

      ROS_INFO("[NavigateWaypointsX] navigating to %lf %lf yaw: %lf", target.x_, target.y_, target.yaw_);
      this->configure<NavigationOrthogonal>(std::make_shared<SbNavigateGlobalPosition>(target.x_, target.y_, target.yaw_));
    }
  }

  sc::result navigateState()
  {
    switch (currentIteration)
    {
    case 1:
    {
      ROS_INFO("transition to ss1");
      auto ev1 = new EvWaypoint1<StNavigateToWaypointsX>();
      this->postEvent(ev1);
    }
    break;
      //return transit<SS1::SsRadialPattern1>();
    case 2:
    {
      ROS_INFO("transition to ss2");
      auto ev2 = new EvWaypoint2<StNavigateToWaypointsX>();
      this->postEvent(ev2);
    }
    break;
      //return transit<SS2::SsRadialPattern2>();
    case 3:
    {
      ROS_INFO("transition to ss3");
      auto ev3 = new EvWaypoint3<StNavigateToWaypointsX>();
      this->postEvent(ev3);
    }
    break;
    case 4:
    {
      ROS_INFO("transition to ss4");
      auto ev4 = new EvWaypoint4<StNavigateToWaypointsX>();
      this->postEvent(ev4);
    }
    break;
    case 5:
    {
      ROS_INFO("transition to ss5");
      auto ev5 = new EvWaypoint5<StNavigateToWaypointsX>();
      this->postEvent(ev5);
    }
    break;
      //return transit<SS3::SsRadialPattern3>();
    default:
      ROS_INFO("error in transition");
      break;
    }

    return forward_event();
  }

  sc::result react(const EvActionSucceded<smacc::SmaccMoveBaseActionClient> &ev)
  {
    ROS_INFO("Waypoints X reaction");

    currentIteration++;
    this->setGlobalSMData("navigation_x_iteration", currentIteration);
    return navigateState();
  }

  sc::result react(const EvKeyPressN<SbKeyboard> &ev)
  {
    currentIteration++;
    this->setGlobalSMData("navigation_x_iteration", currentIteration);
    return navigateState();
  }

  sc::result react(const EvKeyPressP<SbKeyboard> &ev)
  {
    switch (currentIteration)
    {
    case 0:
      return transit<StRotateDegrees2>();
    case 1:
      return transit<StRotateDegrees3>();
    default:
      ROS_INFO("error in transition");
    }

    return forward_event();
  }
};
