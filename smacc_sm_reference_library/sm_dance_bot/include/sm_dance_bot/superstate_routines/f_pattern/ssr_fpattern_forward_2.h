struct SsrFPatternForward2 : smacc::SmaccState<SsrFPatternForward2, SS>
{
  using SmaccState::SmaccState;

  typedef smacc::transition<EvActionSucceeded<smacc::ClMoveBaseZ, OrNavigation>, SsrFPatternStartLoop> reactions;

  static void onDefinition()
  {
  }
  
  void onInitialize()
  {
    auto &superstate = this->context<SS>();
    ROS_INFO("[SsrFpattern] Fpattern rotate: SS current iteration: %d/%d", superstate.iteration_count, superstate.total_iterations());

    this->configure<OrNavigation, CbNavigateForward>(SS::pitch_lenght_meters());
    this->configure<OrTool, CbToolStop>();
  }
};