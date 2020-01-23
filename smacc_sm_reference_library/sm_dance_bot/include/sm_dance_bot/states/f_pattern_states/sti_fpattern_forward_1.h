
namespace sm_dance_bot
{
namespace f_pattern_states
{
template <typename SS>
struct StiFPatternForward1 : public smacc::SmaccState<StiFPatternForward1<SS>, SS>
{
  typedef SmaccState<StiFPatternForward1<SS>, SS> TSti;
  using TSti::SmaccState;
  using TSti::context_type;

  typedef smacc::Transition<EvActionSucceeded<ClMoveBaseZ, OrNavigation>, StiFPatternReturn1<SS>> reactions;

  static void staticConfigure()
  {
     TSti::template configure_orthogonal<OrNavigation, CbNavigateForward>(SS::ray_lenght_meters());
     TSti::template configure_orthogonal<OrLED, CbLEDOn>();
  }

  void runtimeConfiguration()
  {
    auto &superstate = TSti::template context<SS>();
    ROS_INFO("[SsrFpattern] Fpattern rotate: SS current iteration: %d/%d", superstate.iteration_count, SS::total_iterations());
  }
};
}
}