namespace sm_starcraft_ai
{
namespace attack_inner_states
{
// STATE DECLARATION
struct StiAttack1 : smacc::SmaccState<StiAttack1, SS>
{
public:
  using SmaccState::SmaccState;

// TRANSITION TABLE
  typedef mpl::list<

  Transition<EvLoopContinue<StiAttack1>, StiAttack2, CONTINUELOOP>

  >reactions;

// STATE FUNCTIONS
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
    checkWhileLoopConditionAndThrowEvent(&StiAttack1::loopWhileCondition);
  }
};
}
}
