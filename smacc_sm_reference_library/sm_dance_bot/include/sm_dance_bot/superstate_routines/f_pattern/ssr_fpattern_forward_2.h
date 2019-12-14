struct SsrFPatternForward2 : smacc::SmaccState<SsrFPatternForward2, SS>
{
  using SmaccState::SmaccState;

  typedef smacc::transition<EvActionSucceeded<smacc::SmaccMoveBaseActionClient, NavigationOrthogonal>, SsrFPatternStartLoop> reactions;

  static void onDefinition()
  {
  }
  
  void onInitialize()
  {
    auto &superstate = this->context<SS>();
    ROS_INFO("[SsrFpattern] Fpattern rotate: SS current iteration: %d/%d", superstate.iteration_count, superstate.total_iterations());

    this->configure<NavigationOrthogonal, SbNavigateForward>(SS::pitch_lenght_meters());
    this->configure<ToolOrthogonal, SbToolStop>();
  }
};