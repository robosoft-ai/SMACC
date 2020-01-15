namespace sm_dance_bot
{
namespace f_pattern_states
{
template <typename SS>
struct StiFPatternReturn1 : smacc::SmaccState<StiFPatternReturn1<SS>, SS>
{
  typedef SmaccState<StiFPatternReturn1<SS>, SS> TSti;
  using TSti::SmaccState;
  using TSti::context_type;

  typedef smacc::Transition<EvActionSucceeded<ClMoveBaseZ, OrNavigation>, StiFPatternRotate2<SS>> reactions;

  static void onDefinition()
  {
    TSti::template static_configure<OrNavigation, CbUndoPathBackwards>();
    TSti::template static_configure<OrLED, CbLEDOn>();
  }

  void onInitialize()
  {
  }
};
}
}