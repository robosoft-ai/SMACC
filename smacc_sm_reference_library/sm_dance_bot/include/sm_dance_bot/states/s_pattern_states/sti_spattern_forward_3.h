namespace sm_dance_bot
{
namespace s_pattern_states
{
struct StiSPatternForward3 : public smacc::SmaccState<StiSPatternForward3, SS>
{
  using SmaccState::SmaccState;

  typedef mpl::list<smacc::Transition<EvActionSucceeded<ClMoveBaseZ, OrNavigation>, StiSPatternRotate4>,
                    smacc::Transition<EvActionAborted<ClMoveBaseZ, OrNavigation>, StiSPatternRotate3>>
      reactions;

  static void staticConfigure()
  {
    configure_orthogonal<OrNavigation, CbNavigateForward>(SS::pitch1_lenght_meters());
    configure_orthogonal<OrLED, CbLEDOn>();
  }

  void runtimeConfigure()
  {
  }
};
} // namespace s_pattern_states
} // namespace sm_dance_bot