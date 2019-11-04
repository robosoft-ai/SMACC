struct SsrRadialReturn : smacc::SmaccState<SsrRadialReturn, SS>
{
  typedef mpl::list<
            smacc::transition<EvActionSucceded<smacc::SmaccMoveBaseActionClient>, SsrRadialLoopStart, SUCCESS>,
            smacc::transition<EvActionAborted<smacc::SmaccMoveBaseActionClient>, SsrRadialEndPoint, ABORT>
            > reactions;

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