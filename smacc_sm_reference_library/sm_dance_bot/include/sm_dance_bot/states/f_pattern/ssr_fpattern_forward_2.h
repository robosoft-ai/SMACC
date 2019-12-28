namespace fpattern_substates
{
template <typename SS>
struct SsrFPatternForward2 : smacc::SmaccState<SsrFPatternForward2<SS>, SS>
{
  typedef SmaccState<SsrFPatternForward2<SS>, SS> TSsr;
  using TSsr::SmaccState;
  using TSsr::context_type;

  typedef smacc::transition<EvActionSucceeded<ClMoveBaseZ, OrNavigation>, SsrFPatternStartLoop<SS>> reactions;

  static void onDefinition()
  {
  }
  
  void onInitialize()
  {
    auto &superstate = TSsr::template context<SS>();
    ROS_INFO("[SsrFpattern] Fpattern rotate: SS current iteration: %d/%d", superstate.iteration_count, superstate.total_iterations());

    TSsr::template configure<OrNavigation, CbNavigateForward>(SS::pitch_lenght_meters());
    TSsr::template configure<OrTool, CbToolStop>();
  }
};
}