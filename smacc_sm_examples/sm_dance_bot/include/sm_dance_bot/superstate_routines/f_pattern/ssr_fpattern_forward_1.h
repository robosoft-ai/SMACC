struct SsrFPatternForward1 : public smacc::SmaccState<SsrFPatternForward1, SS>
{
  using SmaccState::SmaccState;

  typedef sc::transition<EvActionSucceded<smacc::SmaccMoveBaseActionClient::Result>, SsrFPatternReturn1> reactions;

  void onInitialize()
  {
    auto &superstate = this->context<SS>();
    ROS_INFO("[SsrFpattern] Fpattern rotate: SS current iteration: %d/%d", superstate.iteration_count, superstate.total_iterations);

    this->configure<NavigationOrthogonal>(std::make_shared<SbNavigateForward>(superstate.ray_lenght_meters));
    this->configure<ToolOrthogonal>(std::make_shared<SbToolStart>());
  }
};