struct ssr_radial_rotate: smacc::SmaccState<ssr_radial_rotate,SS>
{
  using SmaccState::SmaccState;

  void onInitialize()
  {
    this->configure<NavigationOrthogonal>(new sb_rotate(10));
    this->configure<ToolOrthogonal>(new sb_tool_stop());
  }
};