struct SsrSPatternLoopEnd : smacc::SmaccState<SsrSPatternLoopEnd, SS>
{
  using SmaccState::SmaccState;
  typedef smacc::transition<EvLoopContinue<SsrSPatternLoopEnd>, SsrSPatternRotate1, CONTINUELOOP> reactions;

  static void onDefinition()
  {

  }

  void onInitialize()
  {
  }

  bool loopCondition()
  {
    auto &superstate = this->context<SS>();
    return ++superstate.iteration_count == superstate.total_iterations();
  }

  void onEntry()
  {
    throwLoopEventFromCondition(&SsrSPatternLoopEnd::loopCondition);
  }
};

