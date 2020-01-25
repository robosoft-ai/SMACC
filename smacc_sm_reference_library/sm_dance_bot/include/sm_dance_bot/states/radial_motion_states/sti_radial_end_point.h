namespace sm_dance_bot
{
namespace radial_motion_states
{
struct StiRadialEndPoint : smacc::SmaccState<StiRadialEndPoint, SS>
{
  using SmaccState::SmaccState;

  typedef mpl::list<
      smacc::Transition<EvActionSucceeded<ClMoveBaseZ, OrNavigation>, StiRadialReturn, SUCCESS>,
      smacc::Transition<EvActionAborted<ClMoveBaseZ, OrNavigation>, StiRadialRotate, ABORT>>
      reactions;

  static void staticConfigure()
  {
    ROS_INFO("ssr radial end point, distance in meters: %lf", SS::ray_length_meters());
    configure_orthogonal<OrNavigation, CbNavigateForward>(SS::ray_length_meters());
    configure_orthogonal<OrLED, CbLEDOff>();
  }

  void runtimeConfigure()
  {
  }
};
} // namespace radial_motion_states
} // namespace sm_dance_bot