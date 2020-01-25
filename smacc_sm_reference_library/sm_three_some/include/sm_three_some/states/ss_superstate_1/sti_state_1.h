namespace sm_three_some
{
namespace ss1_states
{
struct StiState1 : smacc::SmaccState<StiState1, SS>
{
public:
  using SmaccState::SmaccState;

  typedef Transition<EvLoopContinue<StiState1>, StiState2, CONTINUELOOP> reactions;

  //-------------------------------------------------------------------------------
  static void staticConfigure()
  {
  }

  //-------------------------------------------------------------------------------
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
    checkWhileLoopConditionAndThrowEvent(&StiState1::loopWhileCondition);
  }
};
}
}