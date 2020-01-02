namespace sm_dance_bot
{
namespace s_pattern_states
{
struct SsrSPatternForward2 : public smacc::SmaccState<SsrSPatternForward2, SS>
{
  using SmaccState::SmaccState;

  typedef mpl::list<smacc::transition<EvActionSucceeded<ClMoveBaseZ, OrNavigation>, SsrSPatternRotate3>,
                    smacc::transition<EvActionAborted<ClMoveBaseZ, OrNavigation>, SsrSPatternRotate2>>
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