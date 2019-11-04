struct SsrSPatternForward4 : public smacc::SmaccState<SsrSPatternForward4, SS>
{
  using SmaccState::SmaccState;

  typedef mpl::list<smacc::transition<EvActionSucceded<smacc::SmaccMoveBaseActionClient>, SsrSPatternLoopStart>,
                    smacc::transition<EvActionAborted<smacc::SmaccMoveBaseActionClient>, SsrSPatternRotate4>
                    > reactions;

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