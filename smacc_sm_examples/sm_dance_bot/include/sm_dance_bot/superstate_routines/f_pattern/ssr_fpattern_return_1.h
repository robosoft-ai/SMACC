struct SsrFPatternReturn1 : smacc::SmaccState<SsrFPatternReturn1, SS>
{
  using SmaccState::SmaccState;

  typedef sc::transition<EvActionSucceded<smacc::SmaccMoveBaseActionClient::Result>, SsrFPatternRotate2> reactions;

  void onInitialize()
  {
    auto &superstate = this->context<SS>();
    ROS_INFO("[SsrFpattern] Fpattern rotate: SS current iteration: %d/%d", superstate.iteration_count, superstate.total_iterations);

    this->configure<NavigationOrthogonal>(std::make_shared<SbUndoPathBackwards>());
    this->configure<ToolOrthogonal>(std::make_shared<SbToolStart>());
  }
};