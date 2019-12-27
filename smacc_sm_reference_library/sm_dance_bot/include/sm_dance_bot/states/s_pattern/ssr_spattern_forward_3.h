struct SsrSPatternForward3 : public smacc::SmaccState<SsrSPatternForward3, SS>
{
  using SmaccState::SmaccState;

  typedef mpl::list<smacc::transition<EvActionSucceeded<smacc::ClMoveBaseZ, OrNavigation>, SsrSPatternRotate4>,
                    smacc::transition<EvActionAborted<smacc::ClMoveBaseZ, OrNavigation>, SsrSPatternRotate3>
                    > reactions;

  static void onDefinition()
  {
    static_configure<OrNavigation, CbNavigateForward>(SS::pitch2_lenght_meters());
    static_configure<OrTool, CbToolStart>();
  }

  void onInitialize()
  {

  }
};