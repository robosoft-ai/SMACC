namespace sm_dance_bot_3
{
namespace f_pattern_states
{
template <typename SS>
struct StiFPatternForward2 : smacc::SmaccState<StiFPatternForward2<SS>, SS>
{
  typedef SmaccState<StiFPatternForward2<SS>, SS> TSti;
  using TSti::SmaccState;
  using TSti::context_type;

  typedef smacc::Transition<EvActionSucceeded<ClMoveBaseZ, OrNavigation>, StiFPatternStartLoop<SS>> reactions;

  static void staticConfigure()
  {
  }
  
  void runtimeConfiguration()
  {
    auto &superstate = TSti::template context<SS>();
    ROS_INFO("[SsrFpattern] Fpattern rotate: SS current iteration: %d/%d", superstate.iteration_count, superstate.total_iterations());

    TSti::template configure<OrNavigation, CbNavigateForward>(SS::pitch_lenght_meters());
    TSti::template configure<OrLED, CbLEDOff>();
  }
};
}
}