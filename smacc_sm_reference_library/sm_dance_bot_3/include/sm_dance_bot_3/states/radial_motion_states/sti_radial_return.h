namespace sm_dance_bot_3
{
namespace radial_motion_states
{
struct StiRadialReturn : smacc::SmaccState<StiRadialReturn, SS>
{
  typedef mpl::list<
            smacc::Transition<EvActionSucceeded<ClMoveBaseZ, OrNavigation>, StiRadialLoopStart, SUCCESS>,
            smacc::Transition<EvActionAborted<ClMoveBaseZ, OrNavigation>, StiRadialEndPoint, ABORT>
            > reactions;

  using SmaccState::SmaccState;

  static void staticConfigure()
  {
    configure_orthogonal<OrNavigation, CbUndoPathBackwards>();
    configure_orthogonal<OrLED, CbLEDOff>();
  }

  void runtimeConfigure()
  {
  }
};
}
}