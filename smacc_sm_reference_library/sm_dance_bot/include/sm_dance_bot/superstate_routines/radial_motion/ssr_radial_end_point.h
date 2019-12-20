struct SsrRadialEndPoint : smacc::SmaccState<SsrRadialEndPoint, SS>
{
  using SmaccState::SmaccState;

  typedef mpl::list<
              smacc::transition<EvActionSucceeded<smacc::SmaccMoveBaseActionClient, NavigationOrthogonal>, SsrRadialReturn, SUCCESS>,
              smacc::transition<EvActionAborted<smacc::SmaccMoveBaseActionClient, NavigationOrthogonal>, SsrRadialRotate, ABORT>
              > reactions;

  static void onDefinition()
  {
    ROS_INFO("ssr radial end point, distance in meters: %lf", SS::ray_length_meters());
    static_configure<NavigationOrthogonal, CbNavigateForward>(SS::ray_length_meters());
    static_configure<ToolOrthogonal, CbToolStop>();
  }

  void onInitialize()
  {

  }
};
