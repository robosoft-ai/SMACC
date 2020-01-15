namespace sm_dance_bot
{
namespace radial_motion_states
{
struct StiRadialRotate : smacc::SmaccState<StiRadialRotate, SS>
{
  using SmaccState::SmaccState;

  typedef mpl::list<
      smacc::Transition<EvActionSucceeded<ClMoveBaseZ, OrNavigation>, StiRadialEndPoint, SUCCESS>,
      smacc::Transition<EvActionAborted<ClMoveBaseZ, OrNavigation>, StiRadialLoopStart, ABORT>>
      reactions;

  static void onDefinition()
  {
    static_configure<OrNavigation, CbAbsoluteRotate>();
    static_configure<OrLED, CbLEDOff>();
  }

  void onInitialize()
  {
    auto cbAbsRotate = this->getOrthogonal<OrNavigation>()
                           ->getClientBehavior<CbAbsoluteRotate>();

    auto &superstate = this->context<SS>();
    cbAbsRotate->absoluteGoalAngleDegree = superstate.iteration_count * SS::ray_angle_increment_degree();
  }
};
} // namespace radial_motion_states
} // namespace sm_dance_bot