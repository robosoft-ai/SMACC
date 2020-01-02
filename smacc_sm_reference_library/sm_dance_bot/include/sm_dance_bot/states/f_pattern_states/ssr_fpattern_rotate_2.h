namespace sm_dance_bot
{
namespace f_pattern_states
{
template <typename SS>
struct SsrFPatternRotate2 : smacc::SmaccState<SsrFPatternRotate2<SS>, SS>
{
  typedef SmaccState<SsrFPatternRotate2<SS>, SS> TSsr;
  using TSsr::context_type;
  using TSsr::SmaccState;

  typedef smacc::transition<EvActionSucceeded<ClMoveBaseZ, OrNavigation>, SsrFPatternForward2<SS>> reactions;

  static void onDefinition()
  {
    float angle = 0;
    if (SS::direction() == TDirection::LEFT)
      angle = -90;
    else
      angle = 90;

    TSsr::template static_configure<OrNavigation, CbRotate>(angle);
    TSsr::template static_configure<OrLED, CbLEDOff>();
  }

  void onInitialize()
  {
  }
};
} // namespace f_pattern_states
}