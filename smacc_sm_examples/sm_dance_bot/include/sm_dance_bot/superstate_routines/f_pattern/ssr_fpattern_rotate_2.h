struct SsrFPatternRotate2 : smacc::SmaccState<SsrFPatternRotate2, SS>
{
  using SmaccState::SmaccState;

  typedef sc::transition<EvActionSucceded<smacc::SmaccMoveBaseActionClient::Result>, SsrFPatternForward2> reactions;

  void onInitialize()
  {
    auto &superstate = this->context<SS>();
    ROS_INFO("[SsrFpattern] Fpattern rotate: SS current iteration: %d/%d", superstate.iteration_count, superstate.total_iterations);

    float angle = 0;
    if (superstate.direction == TDirection::LEFT)
      angle = -90;
    else
      angle = 90;

    this->configure<NavigationOrthogonal>(std::make_shared<SbRotate>(angle));
    this->configure<ToolOrthogonal>(std::make_shared<SbToolStop>());
  }
};