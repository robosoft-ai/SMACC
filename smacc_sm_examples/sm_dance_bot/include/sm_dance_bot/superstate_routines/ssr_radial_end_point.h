struct ssr_radial_end_point: smacc::SmaccState<ssr_radial_end_point,SS>
{
  using SmaccState::SmaccState;

  typedef sc::transition<EvActionSucceded<smacc::SmaccMoveBaseActionClient::Result>,  ssr_radial_return> reactions; 

  void onInitialize()
  {
    this->configure<NavigationOrthogonal>(new sb_navigate_forward(10));
    this->configure<ToolOrthogonal>(new sb_tool_stop());    
  }

};
