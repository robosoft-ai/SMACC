namespace sm_dance_bot
{
namespace radial_motion_states
{
// STATE DECLARATION
struct StiRadialEndPoint : smacc::SmaccState<StiRadialEndPoint, SS>
{
  using SmaccState::SmaccState;

// TRANSITION TABLE
  typedef mpl::list<
  
  Transition<EvActionSucceeded<ClMoveBaseZ, OrNavigation>, StiRadialReturn, SUCCESS>,
  Transition<EvActionAborted<ClMoveBaseZ, OrNavigation>, StiRadialRotate, ABORT>
  
  >reactions;

// STATE FUNCTIONS
  static void staticConfigure()
  {
    ROS_INFO("ssr radial end point, distance in meters: %lf", SS::ray_length_meters());
    configure_orthogonal<OrNavigation, CbNavigateForward>(SS::ray_length_meters());
    configure_orthogonal<OrLED, CbLEDOn>();
  }

  void runtimeConfigure()
  {
  }
};
} // namespace radial_motion_states
} // namespace sm_dance_bot