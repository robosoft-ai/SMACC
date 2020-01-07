namespace sm_dance_bot_2
{
namespace radial_motion_states
{
using namespace ::sm_dance_bot_2;

struct SsrRadialRotate : smacc::SmaccState<SsrRadialRotate, SS>
{
  using SmaccState::SmaccState;

  typedef mpl::list<
      smacc::transition<EvActionSucceeded<ClMoveBaseZ, OrNavigation>,
                        SsrRadialEndPoint, SUCCESS>,
      smacc::transition<EvActionAborted<ClMoveBaseZ, OrNavigation>,
                        SsrRadialLoopStart, ABORT>>
      reactions;

  static void onDefinition()
  {
    static_configure<OrNavigation, CbAbsoluteRotate>();
  }

  void onInitialize()
  {
    auto cbAbsRotate = this->getOrthogonal<OrNavigation>()
                           ->getClientBehavior<CbAbsoluteRotate>();

    auto& superstate = this->context<SS>();
    cbAbsRotate->absoluteGoalAngleDegree = superstate.iteration_count * SS::ray_angle_increment_degree();
  }
};
} // namespace radial_motion_states
} // namespace sm_dance_bot_2