struct SsrRadialEndPoint : smacc::SmaccState<SsrRadialEndPoint, SS>
{
  using SmaccState::SmaccState;

  typedef mpl::list<
              smacc::transition<EvActionSucceded<smacc::SmaccMoveBaseActionClient>, SsrRadialReturn, SUCCESS>,
              smacc::transition<EvActionAborted<smacc::SmaccMoveBaseActionClient>, SsrRadialRotate, ABORT>
              > reactions;

  static void onDefinition()
  {
    ROS_INFO("ssr radial end point, distance in meters: %lf", SS::ray_length_meters());
    static_configure<NavigationOrthogonal, SbNavigateForward>(SS::ray_length_meters());
    static_configure<ToolOrthogonal, SbToolStop>();
  }

  void onInitialize()
  {

  }
};
