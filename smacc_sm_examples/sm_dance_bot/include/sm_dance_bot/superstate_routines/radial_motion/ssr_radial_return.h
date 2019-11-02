struct SsrRadialReturn : smacc::SmaccState<SsrRadialReturn, SS>
{
  typedef smacc::transition<EvActionSucceded<smacc::SmaccMoveBaseActionClient>, SsrRadialLoopEnd> reactions;

  using SmaccState::SmaccState;

  static void onDefinition()
  {
    static_configure<NavigationOrthogonal, SbUndoPathBackwards>();
    static_configure<ToolOrthogonal, SbToolStop>();
  }

  void onInitialize()
  {
  }
};