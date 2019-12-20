struct SsrFPatternReturn1 : smacc::SmaccState<SsrFPatternReturn1, SS>
{
  using SmaccState::SmaccState;

  typedef smacc::transition<EvActionSucceeded<smacc::SmaccMoveBaseActionClient, NavigationOrthogonal>, SsrFPatternRotate2> reactions;

  static void onDefinition()
  {
    static_configure<NavigationOrthogonal, CbUndoPathBackwards>();
    static_configure<ToolOrthogonal, CbToolStart>();
  }

  void onInitialize()
  {
  }
};