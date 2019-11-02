struct SsrSPatternForward4 : public smacc::SmaccState<SsrSPatternForward4, SS>
{
  using SmaccState::SmaccState;

  typedef smacc::transition<EvActionSucceded<smacc::SmaccMoveBaseActionClient>, SsrSPatternLoopEnd> reactions;

  static void onDefinition()
  {
  }

  void onInitialize()
  {
    auto &superstate = this->context<SS>();

    this->configure<NavigationOrthogonal>(std::make_shared<SbNavigateForward>(SS::pitch1_lenght_meters()));
    this->configure<ToolOrthogonal>(std::make_shared<SbToolStart>());
  }
};