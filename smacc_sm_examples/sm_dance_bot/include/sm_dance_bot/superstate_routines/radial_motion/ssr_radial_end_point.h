struct SsrRadialEndPoint : smacc::SmaccState<SsrRadialEndPoint, SS>
{
  using SmaccState::SmaccState;

  typedef smacc::transition<EvActionSucceded<smacc::SmaccMoveBaseActionClient>, SsrRadialReturn> reactions;

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
