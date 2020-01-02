namespace sm_dance_bot
{
namespace s_pattern_states
{
struct SsrSPatternForward1 : public smacc::SmaccState<SsrSPatternForward1, SS>
{
  using SmaccState::SmaccState;

  typedef mpl::list<smacc::transition<EvActionSucceeded<ClMoveBaseZ, OrNavigation>, SsrSPatternRotate2>,
                    smacc::transition<EvActionAborted<ClMoveBaseZ, OrNavigation>, SsrSPatternRotate1>>
      reactions;

  static void onDefinition()
  {
    static_configure<OrLED, CbLEDOn>();
    static_configure<OrNavigation, CbNavigateForward>(SS::pitch2_lenght_meters());
  }

  void onInitialize()
  {
  }
};
} // namespace s_pattern_states
} // namespace sm_dance_bot