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

  bool loopWhileCondition()
  {
    auto &superstate = this->context<SS>();

    ROS_INFO("Loop start, current iterations: %d, total iterations: %d", superstate.iteration_count, superstate.total_iterations());
    return ++superstate.iteration_count < superstate.total_iterations();
  }

  void onEntry()
  {
    ROS_INFO("LOOP START ON ENTRY");
    throwLoopEventFromCondition(&SsrRadialLoopStart::loopWhileCondition);
  }
};

