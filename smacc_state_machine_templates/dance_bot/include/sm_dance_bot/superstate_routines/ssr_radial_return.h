struct ssr_radial_return: smacc::SmaccState<ssr_radial_return,SS>
{
  typedef sc::transition<smacc::SmaccMoveBaseActionClient::SuccessEv,  ssr_radial_end_point> reactions; 

  using SmaccState::SmaccState;

  void onInitialize()
  {
    this->configure<NavigationOrthogonal>(new sb_undo_path_backwards());
    this->configure<ToolOrthogonal>(new sb_tool_stop());
  }
};