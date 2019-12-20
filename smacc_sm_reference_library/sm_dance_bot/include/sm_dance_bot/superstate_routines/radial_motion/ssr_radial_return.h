struct SsrRadialReturn : smacc::SmaccState<SsrRadialReturn, SS>
{
  typedef mpl::list<
            smacc::transition<EvActionSucceeded<smacc::SmaccMoveBaseActionClient, OrNavigation>, SsrRadialLoopStart, SUCCESS>,
            smacc::transition<EvActionAborted<smacc::SmaccMoveBaseActionClient, OrNavigation>, SsrRadialEndPoint, ABORT>
            > reactions;

  using SmaccState::SmaccState;

  static void onDefinition()
  {
    static_configure<OrNavigation, CbUndoPathBackwards>();
    static_configure<OrTool, CbToolStop>();
  }

  void onInitialize()
  {
  }
};