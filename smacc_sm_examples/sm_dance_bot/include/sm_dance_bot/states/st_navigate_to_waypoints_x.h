
struct st_navigate_to_waypoints_x : smacc::SmaccState<st_navigate_to_waypoints_x, sm_dance_bot>
{
  using SmaccState::SmaccState;

  typedef sc::custom_reaction<smacc::SmaccMoveBaseActionClient::SuccessEv> reactions;

  int currentIteration;

  void onInitialize()
  {
    this->currentIteration = 0;
    // IDEA: this->declarePersistentVariable(currentIteration)
    this->getGlobalSMData("navigation_x_iteration", currentIteration);

    ROS_INFO("current iteration waypoints x: %d", currentIteration);

    std::vector<std::pair<float, float>> waypoints = {
        {1.20, 0.15},
        {1.60, 0.24},
        {2.24, 0.88}};

    auto &target = waypoints[currentIteration];

    this->configure<NavigationOrthogonal>(new sb_navigate_global_position(target.first,target.second));
    this->configure<ToolOrthogonal>(new sb_tool_start());
  }

  sc::result react(const smacc::SmaccMoveBaseActionClient::SuccessEv &ev)
  {
    ROS_INFO("Waypoints X reaction");

    currentIteration++;
    this->setGlobalSMData("navigation_x_iteration", currentIteration);
    
    switch (currentIteration)
    {
      case 1:
        ROS_INFO("transition to ss1");
        return transit<SS1::ss_radial_pattern_1>();
      case 2:
      ROS_INFO("transition to ss2");
        return transit<SS2::ss_radial_pattern_2>();
      case 3:
      ROS_INFO("transition to ss3");
        return transit<SS3::ss_radial_pattern_3>();
      default:
        ROS_INFO("error in transition");
    }
  }
};
