struct st_navigate_to_waypoint_x : smacc::SmaccState<st_navigate_to_waypoint_x, sm_dance_bot>
{
  using SmaccState::SmaccState;

  void onInitialize()
  {
    this->configure<NavigationOrthogonal>(new sb_navigate_global_position(0, 0));
    this->configure<ToolOrthogonal>(new sb_tool_start());
  }
};
