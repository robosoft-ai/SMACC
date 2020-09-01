
namespace sm_ridgeback_floor_coverage_static_1
{
  namespace f_pattern_states
  {
    // STATE DECLARATION
    template <typename SS>
    struct StiFPatternForward1 : public smacc::SmaccState<StiFPatternForward1<SS>, SS>
    {
      typedef SmaccState<StiFPatternForward1<SS>, SS> TSti;
      using TSti::context_type;
      using TSti::SmaccState;

      // TRANSITION TABLE
      typedef mpl::list<

          Transition<EvCbSuccess<CbNavigateForward, OrNavigation>, StiFPatternReturn1<SS>>

          >
          reactions;

      // STATE FUNCTIONS
      static void staticConfigure()
      {
        TSti::template configure_orthogonal<OrNavigation, CbNavigateForward>(SS::ray_lenght_meters());
        TSti::template configure_orthogonal<OrLED, CbLEDOn>();
      }

      void runtimeConfigure()
      {
        auto &superstate = TSti::template context<SS>();
        ROS_INFO("[SsrFpattern] Fpattern rotate: SS current iteration: %d/%d", superstate.iteration_count, SS::total_iterations());
      }
    };
  } // namespace f_pattern_states
} // namespace sm_ridgeback_floor_coverage_static_1