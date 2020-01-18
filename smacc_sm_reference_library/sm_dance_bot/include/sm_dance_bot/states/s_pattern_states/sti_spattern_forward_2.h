namespace sm_dance_bot
{
namespace s_pattern_states
{
struct StiSPatternForward2 : public smacc::SmaccState<StiSPatternForward2, SS>
{
  using SmaccState::SmaccState;

  typedef mpl::list<smacc::Transition<EvActionSucceeded<ClMoveBaseZ, OrNavigation>, StiSPatternRotate3>,
                    smacc::Transition<EvActionAborted<ClMoveBaseZ, OrNavigation>, StiSPatternRotate2>>
      reactions;

  static void onDefinition()
  {
  }

  void onInitialize()
  {
    auto &superstate = this->context<SS>();

    this->configure<OrNavigation, CbNavigateForward>(SS::pitch2_lenght_meters());
    this->configure<OrLED, CbLEDOn>();
  }
};
} // namespace s_pattern_states
} // namespace sm_dance_bot