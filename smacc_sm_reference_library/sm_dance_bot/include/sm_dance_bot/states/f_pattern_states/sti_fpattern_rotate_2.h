namespace sm_dance_bot
{
namespace f_pattern_states
{
template <typename SS>
struct StiFPatternRotate2 : smacc::SmaccState<StiFPatternRotate2<SS>, SS>
{
  typedef SmaccState<StiFPatternRotate2<SS>, SS> TSti;
  using TSti::context_type;
  using TSti::SmaccState;

  typedef smacc::Transition<EvActionSucceeded<ClMoveBaseZ, OrNavigation>, StiFPatternForward2<SS>> reactions;

  static void onDefinition()
  {
    double offset = 7; // for a better behaving
    float angle = 0;
    if (SS::direction() == TDirection::LEFT)
      angle = -90 - offset;
    else
      angle = 90 + offset;

    //TSti::template static_configure<OrNavigation, CbRotate>(angle);
    TSti::template static_configure<OrNavigation, CbAbsoluteRotate>(0 + offset); // absolute horizontal
    TSti::template static_configure<OrLED, CbLEDOff>();
  }

  void onInitialize()
  {
  }
};
} // namespace f_pattern_states
}