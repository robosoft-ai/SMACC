namespace sm_dance_bot_3
{
namespace f_pattern_states
{
template <typename SS>
struct StiFPatternStartLoop : smacc::SmaccState<StiFPatternStartLoop<SS>, SS>
{
  typedef SmaccState<StiFPatternStartLoop<SS>, SS> TSti;
  using TSti::context_type;
  using TSti::SmaccState;

  typedef smacc::Transition<EvLoopContinue<StiFPatternStartLoop<SS>>, StiFPatternRotate1<SS>, CONTINUELOOP> reactions;

  static void staticConfigure()
  {
  }

  bool loopCondition()
  {
    auto &superstate = TSti::template context<SS>();
    return superstate.iteration_count++ < superstate.total_iterations();
  }

  void onEntry()
  {
    TSti::checkWhileLoopConditionAndThrowEvent(&StiFPatternStartLoop<SS>::loopCondition);
  }
};
} // namespace f_pattern_states
} // namespace sm_dance_bot_3