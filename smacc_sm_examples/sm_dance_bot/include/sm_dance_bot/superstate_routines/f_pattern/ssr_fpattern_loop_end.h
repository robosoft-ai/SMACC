struct SsrFPatternEndLoop : smacc::SmaccState<SsrFPatternEndLoop, SS>
{
  using SmaccState::SmaccState;

  typedef smacc::transition<EvLoopContinue<SsrFPatternEndLoop>, SsrFPatternRotate1, CONTINUELOOP> reactions;

  static void onDefinition()
  {
  }
  
  bool loopCondition()
  {
    auto &superstate = this->context<SS>();
    return ++superstate.iteration_count < SS::total_iterations();
  }

  void onEntry()
  {
    throwLoopEventFromCondition(&SsrFPatternEndLoop::loopCondition);
  }
};