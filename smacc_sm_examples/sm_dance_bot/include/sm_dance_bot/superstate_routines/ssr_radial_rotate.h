struct ssr_radial_rotate: smacc::SmaccState<ssr_radial_rotate, SS>
{
  using SmaccState::SmaccState;

  typedef mpl::list<sc::transition<EvActionSucceded<smacc::SmaccMoveBaseActionClient::Result>,  ssr_radial_end_point>> reactions; 

  void onInitialize()
  {
    this->configure<NavigationOrthogonal>(new sb_rotate(45));
    this->configure<ToolOrthogonal>(new sb_tool_stop());
  }
};