struct SsrRadialReturn : smacc::SmaccState<SsrRadialReturn, SS>
{
  typedef mpl::list<
            smacc::transition<EvActionSucceeded<smacc::SmaccMoveBaseActionClient, NavigationOrthogonal>, SsrRadialLoopStart, SUCCESS>,
            smacc::transition<EvActionAborted<smacc::SmaccMoveBaseActionClient, NavigationOrthogonal>, SsrRadialEndPoint, ABORT>
            > reactions;

  using SmaccState::SmaccState;

  static void onDefinition()
  {
    static_configure<NavigationOrthogonal, CbUndoPathBackwards>();
    static_configure<ToolOrthogonal, CbToolStop>();
  }

  void onInitialize()
  {
  }
};