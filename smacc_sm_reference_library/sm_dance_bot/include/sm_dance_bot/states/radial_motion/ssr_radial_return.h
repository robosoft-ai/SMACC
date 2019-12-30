namespace sm_dance_bot
{
namespace radial_motion
{
struct SsrRadialReturn : smacc::SmaccState<SsrRadialReturn, SS>
{
  typedef mpl::list<
            smacc::transition<EvActionSucceeded<ClMoveBaseZ, OrNavigation>, SsrRadialLoopStart, SUCCESS>,
            smacc::transition<EvActionAborted<ClMoveBaseZ, OrNavigation>, SsrRadialEndPoint, ABORT>
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