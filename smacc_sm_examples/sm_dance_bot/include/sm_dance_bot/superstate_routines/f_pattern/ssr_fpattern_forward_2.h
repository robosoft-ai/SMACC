struct SsrFPatternForward2 : smacc::SmaccState<SsrFPatternForward2, SS>
{
  using SmaccState::SmaccState;

  typedef sc::transition<EvActionSucceded<smacc::SmaccMoveBaseActionClient>, SsrFPatternRotate1> reactions;

  void onInitialize()
  {
    auto &superstate = this->context<SS>();
    ROS_INFO("[SsrFpattern] Fpattern rotate: SS current iteration: %d/%d", superstate.iteration_count, superstate.total_iterations);

    if (superstate.iteration_count < superstate.total_iterations)
    {
      this->configure<NavigationOrthogonal>(std::make_shared<SbNavigateForward>(superstate.pitch_lenght_meters));
      this->configure<ToolOrthogonal>(std::make_shared<SbToolStop>());
    }
  }
};