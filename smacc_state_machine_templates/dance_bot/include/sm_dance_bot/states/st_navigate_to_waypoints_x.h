
struct st_navigate_to_waypoints_x : smacc::SmaccState<st_navigate_to_waypoints_x, sm_dance_bot>
{
  using SmaccState::SmaccState;

  typedef sc::custom_reaction<smacc::SmaccMoveBaseActionClient::SuccessEv> reactions;

  int currentIteration;

  void onInitialize()
  {
    int currentIteration = 0;
    // IDEA: this->declarePersistentVariable()
    this->getGlobalSMData("navigation_x_iteration", currentIteration);

    std::vector<std::pair<float, float>> waypoints = {
        {120, 15},
        {160, 24},
        {224, 88}};

    auto &target = waypoints[currentIteration];

    this->configure<NavigationOrthogonal>(new sb_navigate_global_position(0, 0));
    this->configure<ToolOrthogonal>(new sb_tool_start());
  }

  sc::result react(const smacc::SmaccMoveBaseActionClient::SuccessEv &ev)
  {
    switch (currentIteration)
    {
      case 0:
        return transition<SS1::ss_radial_pattern_1>();
      case 1:
        return transition<SS2::ss_radial_pattern_2>();
      case 2:
        return transition<SS3::ss_radial_pattern_3>();
    }
  }

  template <typename T>
  sc::result transition()
  {
    int currentIteration;
    currentIteration++;
    this->setGlobalSMData("navigation_x_iteration", currentIteration);

    return transit<T>();
  }
};
