namespace sm_dance_bot
{
namespace s_pattern_states
{
struct StiSPatternForward4 : public smacc::SmaccState<StiSPatternForward4, SS>
{
  using SmaccState::SmaccState;

  typedef mpl::list<smacc::Transition<EvActionSucceeded<ClMoveBaseZ, OrNavigation>, StiSPatternLoopStart>,
                    smacc::Transition<EvActionAborted<ClMoveBaseZ, OrNavigation>, StiSPatternRotate4>>
      reactions;

  static void onDefinition()
  {
  }

  void onInitialize()
  {
    auto &superstate = this->context<SS>();

    this->configure<OrNavigation, CbNavigateForward>(SS::pitch1_lenght_meters());
    this->configure<OrLED, CbLEDOn>();
  }
};
} // namespace s_pattern_states
} // namespace sm_dance_bot