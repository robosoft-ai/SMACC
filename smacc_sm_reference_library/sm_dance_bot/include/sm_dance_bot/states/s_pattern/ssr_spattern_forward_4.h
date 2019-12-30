namespace sm_dance_bot
{
namespace s_pattern
{
struct SsrSPatternForward4 : public smacc::SmaccState<SsrSPatternForward4, SS>
{
  using SmaccState::SmaccState;

  typedef mpl::list<smacc::transition<EvActionSucceeded<ClMoveBaseZ, OrNavigation>, SsrSPatternLoopStart>,
                    smacc::transition<EvActionAborted<ClMoveBaseZ, OrNavigation>, SsrSPatternRotate4>>
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
} // namespace s_pattern
} // namespace sm_dance_bot