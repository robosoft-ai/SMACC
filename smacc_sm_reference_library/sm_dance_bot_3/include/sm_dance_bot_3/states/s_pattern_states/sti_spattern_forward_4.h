namespace sm_dance_bot_3
{
namespace s_pattern_states
{
struct StiSPatternForward4 : public smacc::SmaccState<StiSPatternForward4, SS>
{
  using SmaccState::SmaccState;

  typedef mpl::list<smacc::Transition<EvActionSucceeded<ClMoveBaseZ, OrNavigation>, StiSPatternLoopStart>,
                    smacc::Transition<EvActionAborted<ClMoveBaseZ, OrNavigation>, StiSPatternRotate4>>
      reactions;

  static void staticConfigure()
  {
  }

  void runtimeConfiguration()
  {
    auto &superstate = this->context<SS>();

    this->configure<OrNavigation, CbNavigateForward>(SS::pitch2_lenght_meters());
    this->configure<OrLED, CbLEDOn>();
  }
};
} // namespace s_pattern_states
} // namespace sm_dance_bot_3