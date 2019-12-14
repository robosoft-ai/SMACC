struct SsrSPatternForward4 : public smacc::SmaccState<SsrSPatternForward4, SS>
{
  using SmaccState::SmaccState;

  typedef mpl::list<smacc::transition<EvActionSucceeded<smacc::SmaccMoveBaseActionClient, NavigationOrthogonal>, SsrSPatternLoopStart>,
                    smacc::transition<EvActionAborted<smacc::SmaccMoveBaseActionClient, NavigationOrthogonal>, SsrSPatternRotate4>
                    > reactions;

  static void onDefinition()
  {
  }

  void onInitialize()
  {
    auto &superstate = this->context<SS>();

    this->configure<NavigationOrthogonal, SbNavigateForward>(SS::pitch1_lenght_meters());
    this->configure<ToolOrthogonal, SbToolStart>();
  }
};