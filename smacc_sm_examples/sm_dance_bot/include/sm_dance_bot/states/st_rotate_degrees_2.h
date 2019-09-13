struct st_rotate_degrees_2: smacc::SmaccState<st_rotate_degrees_2,sm_dance_bot>
{
  using SmaccState::SmaccState;

  typedef sc::transition<smacc::SmaccMoveBaseActionClient::SuccessEv, st_navigate_to_waypoints_x> reactions; 

  void onInitialize()
  {
    this->configure<NavigationOrthogonal>(new sb_rotate(30));
    this->configure<ToolOrthogonal>(new sb_tool_stop());
  }
};
