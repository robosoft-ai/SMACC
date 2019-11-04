struct SsrSPatternLoopStart : smacc::SmaccState<SsrSPatternLoopStart, SS>
{
  using SmaccState::SmaccState;
  typedef mpl::list<smacc::transition<EvLoopContinue<SsrSPatternLoopStart>, SsrSPatternRotate1, CONTINUELOOP>
                    > reactions;

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
    throwLoopEventFromCondition(&SsrSPatternLoopStart::loopCondition);
  }
};

