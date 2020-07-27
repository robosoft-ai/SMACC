namespace sm_ridgeback_floor_coverage_static_1
{
namespace radial_motion_states
{
// STATE DECLARATION
struct StiRadialLoopStart : smacc::SmaccState<StiRadialLoopStart, SS>
{
  using SmaccState::SmaccState;

  // TRANSITION TABLE
  typedef mpl::list<
  
  Transition<EvLoopContinue<StiRadialLoopStart>, StiRadialRotate, CONTINUELOOP> 
  
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
    checkWhileLoopConditionAndThrowEvent(&StiRadialLoopStart::loopWhileCondition);
  }
};
} // namespace radial_motion_states
} // namespace sm_ridgeback_floor_coverage_static_1