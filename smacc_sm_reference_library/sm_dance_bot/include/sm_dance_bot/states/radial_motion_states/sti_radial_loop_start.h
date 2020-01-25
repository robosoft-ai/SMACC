namespace sm_dance_bot
{
namespace radial_motion_states
{
struct StiRadialLoopStart : smacc::SmaccState<StiRadialLoopStart, SS>
{
  using SmaccState::SmaccState;
  typedef smacc::Transition<EvLoopContinue<StiRadialLoopStart>, StiRadialRotate, CONTINUELOOP> reactions;

  static void staticConfigure()
  {
  }

  void runtimeConfigure()
  {
  }

  bool loopWhileCondition()
  {
    auto &superstate = this->context<SS>();

    ROS_INFO("Loop start, current iterations: %d, total iterations: %d", superstate.iteration_count, superstate.total_iterations());
    return superstate.iteration_count++ < superstate.total_iterations();
  }

  void onEntry()
  {
    ROS_INFO("LOOP START ON ENTRY");
    checkWhileLoopConditionAndThrowEvent(&StiRadialLoopStart::loopWhileCondition);
  }
};

} // namespace radial_motion_states
} // namespace sm_dance_bot