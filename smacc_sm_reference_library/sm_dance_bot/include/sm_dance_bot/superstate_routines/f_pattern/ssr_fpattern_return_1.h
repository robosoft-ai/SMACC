struct SsrFPatternReturn1 : smacc::SmaccState<SsrFPatternReturn1, SS>
{
  using SmaccState::SmaccState;

  typedef smacc::transition<EvActionSucceeded<smacc::ClMoveBaseZ, OrNavigation>, SsrFPatternRotate2> reactions;

  static void onDefinition()
  {
    static_configure<OrNavigation, CbUndoPathBackwards>();
    static_configure<OrTool, CbToolStart>();
  }

  void onInitialize()
  {
  }
};