struct SsrRadialLoopEnd : smacc::SmaccState<SsrRadialLoopEnd, SS>
{
  using SmaccState::SmaccState;
  typedef smacc::transition<EvLoopContinue<SsrRadialLoopEnd>, SsrRadialRotate, CONTINUELOOP> reactions;

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
    throwLoopEventFromCondition(&SsrRadialLoopEnd::loopCondition);
  }
};

