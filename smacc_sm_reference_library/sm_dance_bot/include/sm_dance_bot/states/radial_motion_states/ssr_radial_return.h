namespace sm_dance_bot
{
namespace radial_motion_states
{
struct SsrRadialReturn : smacc::SmaccState<SsrRadialReturn, SS>
{
  typedef mpl::list<
            smacc::Transition<EvActionSucceeded<ClMoveBaseZ, OrNavigation>, SsrRadialLoopStart, SUCCESS>,
            smacc::Transition<EvActionAborted<ClMoveBaseZ, OrNavigation>, SsrRadialEndPoint, ABORT>
            > reactions;

  using SmaccState::SmaccState;

  static void onDefinition()
  {
    static_configure<OrNavigation, CbUndoPathBackwards>();
    static_configure<OrLED, CbLEDOff>();
  }

  void onInitialize()
  {
  }
};
}
}