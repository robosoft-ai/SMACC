namespace sm_dance_bot_2 {
namespace radial_motion_states {

struct StiRadialReturn : smacc::SmaccState<StiRadialReturn, SS> {
  typedef mpl::list<
      Transition<EvActionSucceeded<ClMoveBaseZ, OrNavigation>,StiRadialLoopStart, SUCCESS>,                  
      Transition<EvActionAborted<ClMoveBaseZ, OrNavigation>,StiRadialEndPoint, ABORT>>
      reactions;

  using SmaccState::SmaccState;

  static void onDefinition() {
    configure_orthogonal<OrNavigation, CbUndoPathBackwards>();
  }

  void onInitialize() {}
};
}
}