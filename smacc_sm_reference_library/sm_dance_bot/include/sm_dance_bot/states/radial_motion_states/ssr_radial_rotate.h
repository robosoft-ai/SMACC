namespace sm_dance_bot
{
namespace radial_motion_states
{
struct SsrRadialRotate : smacc::SmaccState<SsrRadialRotate, SS>
{
  using SmaccState::SmaccState;

  typedef mpl::list<
      smacc::Transition<EvActionSucceeded<ClMoveBaseZ, OrNavigation>, SsrRadialEndPoint, SUCCESS>,
      smacc::Transition<EvActionAborted<ClMoveBaseZ, OrNavigation>, SsrRadialLoopStart, ABORT>>
      reactions;

  static void onDefinition()
  {
    static_configure<OrNavigation, CbRotate>(SS::ray_angle_increment_degree());
    static_configure<OrLED, CbLEDOff>();
  }

  void onInitialize()
  {
  }
};
} // namespace radial_motion_states
} // namespace sm_dance_bot