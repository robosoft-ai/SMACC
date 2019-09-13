struct st_rotate_degrees_4: smacc::SmaccState<st_rotate_degrees_4,sm_dance_bot>
{
  using SmaccState::SmaccState;

  typedef sc::transition<smacc::SmaccMoveBaseActionClient::SuccessEv, st_navigate_reverse_2> reactions;

  void onInitialize()
  {
    this->configure<NavigationOrthogonal>(new sb_rotate(30));
    this->configure<ToolOrthogonal>(new sb_tool_stop());
  }
};
