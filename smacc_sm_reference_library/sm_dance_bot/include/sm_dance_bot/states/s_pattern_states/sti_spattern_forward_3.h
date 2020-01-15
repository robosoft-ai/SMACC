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

  static void onDefinition()
  {
    static_configure<OrNavigation, CbNavigateForward>(SS::pitch2_lenght_meters());
    static_configure<OrLED, CbLEDOn>();
  }

  void onInitialize()
  {
  }
};
} // namespace s_pattern_states
} // namespace sm_dance_bot