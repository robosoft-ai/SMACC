struct SsrRadialRotate : smacc::SmaccState<SsrRadialRotate, SS>
{
  using SmaccState::SmaccState;

  typedef mpl::list<
              smacc::transition<EvActionSucceded<smacc::SmaccMoveBaseActionClient>, SsrRadialEndPoint, SUCCESS>,
              smacc::transition<EvActionAborted<smacc::SmaccMoveBaseActionClient>, SsrRadialLoopStart, ABORT>
              > reactions;

  static void onDefinition()
  {
    static_configure<NavigationOrthogonal, SbRotate>(SS::ray_angle_increment_degree());
    static_configure<ToolOrthogonal, SbToolStop>();
  }

  void onInitialize()
  {
  }
};