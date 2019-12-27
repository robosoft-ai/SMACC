struct SsrSPatternForward1 : public smacc::SmaccState<SsrSPatternForward1, SS>
{
  using SmaccState::SmaccState;

  typedef mpl::list<smacc::transition<EvActionSucceeded<smacc::ClMoveBaseZ, OrNavigation>, SsrSPatternRotate2>,
                    smacc::transition<EvActionAborted<smacc::ClMoveBaseZ, OrNavigation>, SsrSPatternRotate1>
                    > reactions;

  static void onDefinition()
  {
    static_configure<OrTool, CbToolStart>();
    static_configure<OrNavigation, CbNavigateForward>(SS::pitch2_lenght_meters());
  }

  void onInitialize()
  {
  }
};