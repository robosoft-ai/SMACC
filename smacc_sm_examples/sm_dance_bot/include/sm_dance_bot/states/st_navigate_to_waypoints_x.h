
struct StNavigateToWaypointsX : smacc::SmaccState<StNavigateToWaypointsX, SmDanceBot>
{
  using SmaccState::SmaccState;

  typedef mpl::list<
              // Expected event
              sc::custom_reaction<EvActionSucceded<smacc::SmaccMoveBaseActionClient::Result>>,

              // Keyboard event
              sc::custom_reaction<EvKeyPressN<SbKeyboard>>,
              sc::custom_reaction<EvKeyPressP<SbKeyboard>>,

              // Error events
              sc::transition<smacc::EvTopicMessageTimeout<LidarSensor>, StAcquireSensors>,
              sc::transition<EvActionAborted<smacc::SmaccMoveBaseActionClient::Result>, StNavigateToWaypointsX>
          > reactions;

  int currentIteration;

  void onInitialize()
  {
    this->currentIteration = 0;

    // IDEA: this->declarePersistentVariable(&currentIteration)
    this->getGlobalSMData("navigation_x_iteration", currentIteration);

    ROS_INFO("current iteration waypoints x: %d", currentIteration);

    std::vector<std::pair<float, float>> waypoints = {
        {1.20, 0.15},
        {1.60, 0.24},
        {2.24, 0.88}};

    auto &target = waypoints[currentIteration];

    this->configure<NavigationOrthogonal>(new SbNavigateGlobalPosition(target.first, target.second));
    this->configure<ToolOrthogonal>(new SbToolStart());
    this->configure<KeyboardOrthogonal>(new SbKeyboard());
  }

  sc::result navigateState()
  {
    switch (currentIteration)
    {
    case 1:
      ROS_INFO("transition to ss1");
      return transit<SS1::SsRadialPattern1>();
    case 2:
      ROS_INFO("transition to ss2");
      return transit<SS2::SsRadialPattern2>();
    case 3:
      ROS_INFO("transition to ss3");
      return transit<SS3::SsRadialPattern3>();
    default:
      ROS_INFO("error in transition");
    }
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
  }
};
