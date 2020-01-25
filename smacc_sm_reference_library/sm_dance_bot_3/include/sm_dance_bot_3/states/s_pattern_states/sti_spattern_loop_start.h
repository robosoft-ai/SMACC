namespace sm_dance_bot_3
{
namespace s_pattern_states
{
struct StiSPatternLoopStart : smacc::SmaccState<StiSPatternLoopStart, SS>
{
  using SmaccState::SmaccState;
  typedef mpl::list<smacc::Transition<EvLoopContinue<StiSPatternLoopStart>, StiSPatternRotate1, CONTINUELOOP>> reactions;

  static void staticConfigure()
  {
  }

  void runtimeConfigure()
  {
  }

  bool loopCondition()
  {
    auto &superstate = this->context<SS>();
    return superstate.iteration_count++ < superstate.total_iterations();
  }

  void onEntry()
  {
    checkWhileLoopConditionAndThrowEvent(&StiSPatternLoopStart::loopCondition);
  }
};

} // namespace s_pattern_states
} // namespace sm_dance_bot_3