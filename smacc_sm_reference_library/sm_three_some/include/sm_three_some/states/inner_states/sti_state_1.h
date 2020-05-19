namespace sm_three_some
{
namespace inner_states
{
// STATE DECLARATION
struct StiState1 : smacc::SmaccState<StiState1, SS>
{
public:
  using SmaccState::SmaccState;

// TRANSITION TABLE
  typedef mpl::list<
  
  Transition<EvLoopContinue<StiState1>, StiState2, CONTINUELOOP> 
  
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
    checkWhileLoopConditionAndThrowEvent(&StiState1::loopWhileCondition);
  }
  
  void onExit()
  {
    ROS_INFO("On Exit!");
  }
  
};
}
}