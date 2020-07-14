namespace sm_dance_bot_2
{
namespace radial_motion_states
{
// STATE DECLARATION
struct StiRadialReturn : smacc::SmaccState<StiRadialReturn, SS>
{
  using SmaccState::SmaccState;
  
  // TRANSITION TABLE
  typedef mpl::list<
  
  Transition<EvActionSucceeded<ClMoveBaseZ, OrNavigation>, StiRadialLoopStart, SUCCESS>,
  Transition<EvActionAborted<ClMoveBaseZ, OrNavigation>, StiRadialEndPoint, ABORT>
  
  >reactions;

  // STATE FUNCTIONS
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