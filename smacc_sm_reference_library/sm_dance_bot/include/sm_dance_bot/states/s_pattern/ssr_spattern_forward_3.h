namespace sm_dance_bot
{
namespace s_pattern
{
struct SsrSPatternForward3 : public smacc::SmaccState<SsrSPatternForward3, SS>
{
  using SmaccState::SmaccState;

  typedef mpl::list<smacc::transition<EvActionSucceeded<ClMoveBaseZ, OrNavigation>, SsrSPatternRotate4>,
                    smacc::transition<EvActionAborted<ClMoveBaseZ, OrNavigation>, SsrSPatternRotate3>>
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
} // namespace s_pattern
} // namespace sm_dance_bot