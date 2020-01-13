namespace sm_dance_bot
{
namespace f_pattern_states
{
template <typename SS>
struct SsrFPatternStartLoop : smacc::SmaccState<SsrFPatternStartLoop<SS>, SS>
{
  typedef SmaccState<SsrFPatternStartLoop<SS>, SS> TSsr;
  using TSsr::context_type;
  using TSsr::SmaccState;

  typedef smacc::Transition<EvLoopContinue<SsrFPatternStartLoop<SS>>, SsrFPatternRotate1<SS>, CONTINUELOOP> reactions;

  static void onDefinition()
  {
  }

  bool loopCondition()
  {
    auto &superstate = TSsr::template context<SS>();
    return superstate.iteration_count++ < superstate.total_iterations();
  }

  void onEntry()
  {
    TSsr::checkWhileLoopConditionAndThrowEvent(&SsrFPatternStartLoop<SS>::loopCondition);
  }
};
} // namespace f_pattern_states
} // namespace sm_dance_bot