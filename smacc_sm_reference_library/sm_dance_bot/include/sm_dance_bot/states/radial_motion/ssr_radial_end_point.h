namespace sm_dance_bot
{
namespace radial_motion
{
struct SsrRadialEndPoint : smacc::SmaccState<SsrRadialEndPoint, SS>
{
  using SmaccState::SmaccState;

  typedef mpl::list<
      smacc::transition<EvActionSucceeded<ClMoveBaseZ, OrNavigation>, SsrRadialReturn, SUCCESS>,
      smacc::transition<EvActionAborted<ClMoveBaseZ, OrNavigation>, SsrRadialRotate, ABORT>>
      reactions;

  static void onDefinition()
  {
    ROS_INFO("ssr radial end point, distance in meters: %lf", SS::ray_length_meters());
    static_configure<OrNavigation, CbNavigateForward>(SS::ray_length_meters());
    static_configure<OrLED, CbLEDOff>();
  }

  void onInitialize()
  {
  }
};
} // namespace radial_motion
} // namespace sm_dance_bot