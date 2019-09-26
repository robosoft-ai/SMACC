

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

//-------------------------------------------------
struct StNavigateToWaypointsX : smacc::SmaccState<StNavigateToWaypointsX, SmDanceBot>
{
  using SmaccState::SmaccState;

  typedef mpl::list<

      // Expected event
      sc::custom_reaction<EvActionSucceded<smacc::SmaccMoveBaseActionClient::Result>>,

      sc::transition<EvWaypoint1, SS1::SsRadialPattern1>,
      sc::transition<EvWaypoint2, SS2::SsRadialPattern2>,
      sc::transition<EvWaypoint3, SS3::SsRadialPattern3>,

      // Keyboard event
      sc::custom_reaction<EvKeyPressN<SbKeyboard>>,
      sc::custom_reaction<EvKeyPressP<SbKeyboard>>,

      // Error events
      sc::transition<smacc::EvTopicMessageTimeout<LidarSensor>, StAcquireSensors>,
      sc::transition<EvActionAborted<smacc::SmaccMoveBaseActionClient::Result>, StNavigateToWaypointsX>>
      reactions;

  int currentIteration;

  void onInitialize()
  {
    this->currentIteration = 0;

    // IDEA: this->declarePersistentVariable(&currentIteration)
    this->getGlobalSMData("navigation_x_iteration", currentIteration);

    ROS_INFO("current iteration waypoints x: %d", currentIteration);

    std::vector<std::pair<float, float>> waypoints = {
        {2.20, 0.35},
        {2.60, 0.64},
        {4.24, 1.68},
        {-1.24, -2.68}};

    if (currentIteration > waypoints.size())
    {
      ROS_FATAL("[WaypointsX] Out of range of waypoint list. Cannot go to waypoint %d.", this->currentIteration);
      this->throwFinishEvent();
    }
    else
    {
      auto &target = waypoints[currentIteration];

      this->configure<NavigationOrthogonal>(std::make_shared<SbNavigateGlobalPosition>(target.first, target.second));
      this->configure<ToolOrthogonal>(std::make_shared<SbToolStart>());
      this->configure<KeyboardOrthogonal>(std::make_shared<SbKeyboard>());
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
      //return transit<SS3::SsRadialPattern3>();
    default:
      ROS_INFO("error in transition");
      break;
    }

    return forward_event();
  }

  sc::result react(const EvActionSucceded<smacc::SmaccMoveBaseActionClient::Result> &ev)
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
