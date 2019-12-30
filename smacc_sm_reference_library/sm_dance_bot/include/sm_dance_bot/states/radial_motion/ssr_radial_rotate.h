namespace sm_dance_bot
{
namespace radial_motion
{
struct SsrRadialRotate : smacc::SmaccState<SsrRadialRotate, SS>
{
  using SmaccState::SmaccState;

  typedef mpl::list<
      smacc::transition<EvActionSucceeded<ClMoveBaseZ, OrNavigation>, SsrRadialEndPoint, SUCCESS>,
      smacc::transition<EvActionAborted<ClMoveBaseZ, OrNavigation>, SsrRadialLoopStart, ABORT>>
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
} // namespace radial_motion
} // namespace sm_dance_bot