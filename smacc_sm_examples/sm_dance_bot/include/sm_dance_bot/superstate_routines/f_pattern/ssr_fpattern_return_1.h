struct SsrFPatternReturn1 : smacc::SmaccState<SsrFPatternReturn1, SS>
{
  using SmaccState::SmaccState;

  typedef smacc::transition<EvActionSucceded<smacc::SmaccMoveBaseActionClient>, SsrFPatternRotate2> reactions;

  static void onDefinition()
  {
    static_configure<NavigationOrthogonal, SbUndoPathBackwards>();
    static_configure<ToolOrthogonal, SbToolStart>();
  }

  void onInitialize()
  {
  }
};