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

  static void staticConfigure()
  {
    TSti::template configure_orthogonal<OrNavigation, CbUndoPathBackwards>();
    TSti::template configure_orthogonal<OrLED, CbLEDOn>();
  }

  void runtimeConfigure()
  {
  }
};
}
}