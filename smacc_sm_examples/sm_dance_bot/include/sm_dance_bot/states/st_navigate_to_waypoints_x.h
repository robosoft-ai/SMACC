

struct EvWaypoint1 : sc::event<EvWaypoint1>
{
};

struct EvWaypoint2 : sc::event<EvWaypoint2>
{
};

struct EvWaypoint3 : sc::event<EvWaypoint3>
{
};

struct EvWaypoint4 : sc::event<EvWaypoint4>
{
};

struct EvWaypoint5 : sc::event<EvWaypoint5>
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
struct StNavigateToWaypointsX : smacc::SmaccState<StNavigateToWaypointsX, SmDanceBot>
{
  using SmaccState::SmaccState;

  typedef mpl::list<

      // Expected event
      sc::custom_reaction<EvActionSucceded<smacc::SmaccMoveBaseActionClient>>,

      smacc::transition<EvWaypoint1, SS1::SsRadialPattern1>,
      smacc::transition<EvWaypoint2, SS2::SsRadialPattern2>,
      smacc::transition<EvWaypoint3, SS3::SsRadialPattern3>,
      smacc::transition<EvWaypoint4, SS4::SsFPattern1>,
      smacc::transition<EvWaypoint5, SS5::SsSPattern1>,

      // Keyboard event
      sc::custom_reaction<EvKeyPressN<SbKeyboard>>,
      sc::custom_reaction<EvKeyPressP<SbKeyboard>>,

      // Error events
      smacc::transition<smacc::EvTopicMessageTimeout<SbLidarSensor>, StAcquireSensors>,
      smacc::transition<EvActionAborted<smacc::SmaccMoveBaseActionClient>, StNavigateToWaypointsX>>
      reactions;

  int currentIteration;

  static void onDefinition()
  {
    static_configure<ToolOrthogonal, SbToolStart>();
    static_configure<KeyboardOrthogonal, SbKeyboard>();
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
      auto ev1 = new EvWaypoint1();
      this->postEvent(ev1);
    }
    break;
      //return transit<SS1::SsRadialPattern1>();
    case 2:
    {
      ROS_INFO("transition to ss2");
      auto ev2 = new EvWaypoint2();
      this->postEvent(ev2);
    }
    break;
      //return transit<SS2::SsRadialPattern2>();
    case 3:
    {
      ROS_INFO("transition to ss3");
      auto ev3 = new EvWaypoint3();
      this->postEvent(ev3);
    }
    break;
    case 4:
    {
      ROS_INFO("transition to ss4");
      auto ev4 = new EvWaypoint4();
      this->postEvent(ev4);
    }
    break;
    case 5:
    {
      ROS_INFO("transition to ss5");
      auto ev5 = new EvWaypoint5();
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
