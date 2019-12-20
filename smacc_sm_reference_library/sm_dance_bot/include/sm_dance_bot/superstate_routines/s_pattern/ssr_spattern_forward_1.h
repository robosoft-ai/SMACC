struct SsrSPatternForward1 : public smacc::SmaccState<SsrSPatternForward1, SS>
{
  using SmaccState::SmaccState;

  typedef mpl::list<smacc::transition<EvActionSucceeded<smacc::SmaccMoveBaseActionClient, NavigationOrthogonal>, SsrSPatternRotate2>,
                    smacc::transition<EvActionAborted<smacc::SmaccMoveBaseActionClient, NavigationOrthogonal>, SsrSPatternRotate1>
                    > reactions;

  static void onDefinition()
  {
    static_configure<ToolOrthogonal, CbToolStart>();
    static_configure<NavigationOrthogonal, CbNavigateForward>(SS::pitch2_lenght_meters());
  }

  void onInitialize()
  {
  }
};