struct SsrRadialLoopStart : smacc::SmaccState<SsrRadialLoopStart, SS>
{
  using SmaccState::SmaccState;
  typedef smacc::transition<EvLoopContinue<SsrRadialLoopStart>, SsrRadialRotate, CONTINUELOOP> reactions;

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
    throwLoopEventFromCondition(&SsrRadialLoopStart::loopCondition);
  }
};

