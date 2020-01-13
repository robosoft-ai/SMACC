namespace sm_three_some
{
namespace ss1_states
{
struct Ssr1 : smacc::SmaccState<Ssr1, SS>
{
public:
  using SmaccState::SmaccState;

  typedef smacc::Transition<EvLoopContinue<Ssr1>, Ssr2, CONTINUELOOP> reactions;

  //-------------------------------------------------------------------------------
  static void onDefinition()
  {
  }

  //-------------------------------------------------------------------------------
  void onInitialize()
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
    checkWhileLoopConditionAndThrowEvent(&Ssr1::loopWhileCondition);
  }
};
}
}