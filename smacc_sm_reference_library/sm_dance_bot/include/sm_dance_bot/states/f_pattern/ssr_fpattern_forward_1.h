
namespace fpattern_substates
{
template <typename SS>
struct SsrFPatternForward1 : public smacc::SmaccState<SsrFPatternForward1<SS>, SS>
{
  typedef SmaccState<SsrFPatternForward1<SS>, SS> TSsr;
  using TSsr::SmaccState;
  using TSsr::context_type;

  typedef smacc::transition<EvActionSucceeded<ClMoveBaseZ, OrNavigation>, SsrFPatternReturn1<SS>> reactions;

  static void onDefinition()
  {
     TSsr::template static_configure<OrNavigation, CbNavigateForward>(SS::ray_lenght_meters());
     TSsr::template static_configure<OrTool, CbToolStart>();
  }

  void onInitialize()
  {
    auto &superstate = TSsr::template context<SS>();
    ROS_INFO("[SsrFpattern] Fpattern rotate: SS current iteration: %d/%d", superstate.iteration_count, SS::total_iterations());
  }
};
}