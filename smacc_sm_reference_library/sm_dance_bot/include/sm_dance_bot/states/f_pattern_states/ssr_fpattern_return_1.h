namespace sm_dance_bot
{
namespace f_pattern_states
{
template <typename SS>
struct SsrFPatternReturn1 : smacc::SmaccState<SsrFPatternReturn1<SS>, SS>
{
  typedef SmaccState<SsrFPatternReturn1<SS>, SS> TSsr;
  using TSsr::SmaccState;
  using TSsr::context_type;

  typedef smacc::transition<EvActionSucceeded<ClMoveBaseZ, OrNavigation>, SsrFPatternRotate2<SS>> reactions;

  static void onDefinition()
  {
    TSsr::template static_configure<OrNavigation, CbUndoPathBackwards>();
    TSsr::template static_configure<OrLED, CbLEDOn>();
  }

  void onInitialize()
  {
  }
};
}
}