namespace sm_dance_bot
{
namespace f_pattern_states
{
template <typename SS>
struct SsrFPatternStartLoop : smacc::SmaccState<SsrFPatternStartLoop<SS>, SS>
{
  typedef SmaccState<SsrFPatternStartLoop<SS>, SS> TSsr;
  using TSsr::SmaccState;
  using TSsr::context_type;

  typedef smacc::Transition<EvLoopContinue<SsrFPatternStartLoop<SS>>, SsrFPatternRotate1<SS>, CONTINUELOOP> reactions;

  static void onDefinition()
  {
  }
  
  bool loopCondition()
  {
    auto &superstate = TSsr::template context<SS>();
    return ++superstate.iteration_count < SS::total_iterations();
  }

  void onEntry()
  {
    TSsr::throwLoopEventFromCondition(&SsrFPatternStartLoop<SS>::loopCondition);
  }
};
}
}