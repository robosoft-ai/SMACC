struct st_rotate_degrees_4: smacc::SmaccState<st_rotate_degrees_4,sm_dance_bot>
{
  using SmaccState::SmaccState;

  void onInitialize()
  {
    this->configure<NavigationOrthogonal>(new sb_rotate(740));
    this->configure<ToolOrthogonal>(new sb_tool_stop());
  }
};
