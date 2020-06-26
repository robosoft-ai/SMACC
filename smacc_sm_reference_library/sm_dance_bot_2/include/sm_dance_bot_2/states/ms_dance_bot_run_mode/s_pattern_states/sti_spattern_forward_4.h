namespace sm_dance_bot_2
{
namespace s_pattern_states
{
// STATE DECLARATION
struct StiSPatternForward4 : public smacc::SmaccState<StiSPatternForward4, SS>
{
  using SmaccState::SmaccState;

// TRANSITION TABLE
  typedef mpl::list<
  
  Transition<EvActionSucceeded<ClMoveBaseZ, OrNavigation>, StiSPatternLoopStart>,
  Transition<EvActionAborted<ClMoveBaseZ, OrNavigation>, StiSPatternRotate4>
  
  >reactions;

// STATE FUNCTIONS
  static void staticConfigure()
  {
  }

  void runtimeConfigure()
  {
    auto &superstate = this->context<SS>();

    this->configure<OrNavigation, CbNavigateForward>(SS::pitch2_lenght_meters());
    this->configure<OrLED, CbLEDOn>();
  }
};
} // namespace s_pattern_states
} // namespace sm_dance_bot_2