namespace sm_respira_1
{
namespace cmv_cycle_inner_states
{
// STATE DECLARATION
struct StiCMVCycleLoop : smacc::SmaccState<StiCMVCycleLoop, SsCMVCycle>
{
public:
  using SmaccState::SmaccState;

  // TRANSITION TABLE
  typedef mpl::list<

      Transition<EvLoopContinue<StiCMVCycleLoop>, StiCMVCycleInspire, CONTINUELOOP>

      >
      reactions;

  // STATE FUNCTIONS
  static void staticConfigure()
  {
  }

  void runtimeConfigure()
  {
  }

  bool loopWhileCondition()
  {
    auto &superstate = this->context<SsCMVCycle>();

    ROS_INFO("Loop start, current iterations: %d, total iterations: %d", superstate.iteration_count,
             superstate.total_iterations());
    return superstate.iteration_count++ < superstate.total_iterations();
  }

  void onEntry()
  {
    ROS_INFO("LOOP START ON ENTRY");
    checkWhileLoopConditionAndThrowEvent(&StiCMVCycleLoop::loopWhileCondition);
  }
};
}  // namespace cmv_cycle_inner_states
}  // namespace sm_respira_1
