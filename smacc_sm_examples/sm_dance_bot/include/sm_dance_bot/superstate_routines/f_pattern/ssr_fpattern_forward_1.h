struct SsrFPatternForward1 : public smacc::SmaccState<SsrFPatternForward1, SS>
{
  using SmaccState::SmaccState;

  typedef smacc::transition<EvActionSucceded<smacc::SmaccMoveBaseActionClient>, SsrFPatternReturn1> reactions;

  static void onDefinition()
  {
    static_configure<NavigationOrthogonal, SbNavigateForward>(SS::ray_lenght_meters());
    static_configure<ToolOrthogonal, SbToolStart>();
  }

  void onInitialize()
  {
    auto &superstate = this->context<SS>();
    ROS_INFO("[SsrFpattern] Fpattern rotate: SS current iteration: %d/%d", superstate.iteration_count, SS::total_iterations());
  }
};