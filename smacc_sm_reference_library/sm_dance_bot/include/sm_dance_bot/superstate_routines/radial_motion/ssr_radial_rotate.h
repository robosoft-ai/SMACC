struct SsrRadialRotate : smacc::SmaccState<SsrRadialRotate, SS>
{
  using SmaccState::SmaccState;

  typedef mpl::list<
              smacc::transition<EvActionSucceeded<smacc::SmaccMoveBaseActionClient, OrNavigation>, SsrRadialEndPoint, SUCCESS>,
              smacc::transition<EvActionAborted<smacc::SmaccMoveBaseActionClient, OrNavigation>, SsrRadialLoopStart, ABORT>
              > reactions;

  static void onDefinition()
  {
    static_configure<OrNavigation, CbRotate>(SS::ray_angle_increment_degree());
    static_configure<OrTool, CbToolStop>();
  }

  void onInitialize()
  {
  }
};