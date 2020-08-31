namespace sm_dance_bot_2
{
namespace s_pattern_states
{
// STATE DECLARATION
struct StiSPatternForward1 : public smacc::SmaccState<StiSPatternForward1, SS>
{
  using SmaccState::SmaccState;

// TRANSITION TABLE
  typedef mpl::list<
  
  Transition<EvActionSucceeded<ClMoveBaseZ, OrNavigation>, StiSPatternRotate2>,
  Transition<EvActionAborted<ClMoveBaseZ, OrNavigation>, StiSPatternRotate1>
  
  >reactions;

// STATE FUNCTIONS
  static void staticConfigure()
  {
    configure_orthogonal<OrLED, CbLEDOn>();
    configure_orthogonal<OrNavigation, CbNavigateForward>(SS::pitch1_lenght_meters());
  }

  void runtimeConfigure()
  {
  }
};
} // namespace s_pattern_states
} // namespace sm_dance_bot_2