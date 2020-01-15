namespace sm_dance_bot
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