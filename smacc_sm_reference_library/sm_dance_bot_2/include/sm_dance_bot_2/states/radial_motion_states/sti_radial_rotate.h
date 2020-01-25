namespace sm_dance_bot_2
{
namespace radial_motion_states
{
using namespace ::sm_dance_bot_2;

struct StiRadialRotate : smacc::SmaccState<StiRadialRotate, SS>
{
  using SmaccState::SmaccState;

  typedef mpl::list<
      Transition<EvActionSucceeded<ClMoveBaseZ, OrNavigation>,StiRadialEndPoint, SUCCESS>,
      Transition<EvActionAborted<ClMoveBaseZ, OrNavigation>,StiRadialLoopStart, ABORT>>
      reactions;

  static void staticConfigure()
  {
    configure_orthogonal<OrNavigation, CbAbsoluteRotate>();
  }

  void runtimeConfigure()
  {
    auto cbAbsRotate = this->getOrthogonal<OrNavigation>()
                           ->getClientBehavior<CbAbsoluteRotate>();

    auto& superstate = this->context<SS>();
    cbAbsRotate->absoluteGoalAngleDegree = superstate.iteration_count * superstate.ray_angle_increment_degree();
  }
};
} // namespace radial_motion_states
} // namespace sm_dance_bot_2