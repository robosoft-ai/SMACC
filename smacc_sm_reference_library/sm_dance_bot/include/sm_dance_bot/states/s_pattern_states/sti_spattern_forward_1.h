namespace sm_dance_bot
{
namespace s_pattern_states
{
struct StiSPatternForward1 : public smacc::SmaccState<StiSPatternForward1, SS>
{
  using SmaccState::SmaccState;

  typedef mpl::list<smacc::Transition<EvActionSucceeded<ClMoveBaseZ, OrNavigation>, StiSPatternRotate2>,
                    smacc::Transition<EvActionAborted<ClMoveBaseZ, OrNavigation>, StiSPatternRotate1>>
      reactions;

  static void staticConfigure()
  {
    configure_orthogonal<OrLED, CbLEDOn>();
    configure_orthogonal<OrNavigation, CbNavigateForward>(SS::pitch1_lenght_meters());
  }

  void runtimeConfiguration()
  {
  }
};
} // namespace s_pattern_states
} // namespace sm_dance_bot