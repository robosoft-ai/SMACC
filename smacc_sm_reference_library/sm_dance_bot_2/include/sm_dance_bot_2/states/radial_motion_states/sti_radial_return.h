namespace sm_dance_bot_2 {
namespace radial_motion_states {

struct StiRadialReturn : smacc::SmaccState<StiRadialReturn, SS> {
  typedef mpl::list<
      smacc::Transition<EvActionSucceeded<ClMoveBaseZ, OrNavigation>,
                        StiRadialLoopStart, SUCCESS>,
                        
      smacc::Transition<EvActionAborted<ClMoveBaseZ, OrNavigation>,
                        StiRadialEndPoint, ABORT>>
      reactions;

  using SmaccState::SmaccState;

  static void onDefinition() {
    static_configure<OrNavigation, CbUndoPathBackwards>();
  }

  void onInitialize() {}
};
}
}