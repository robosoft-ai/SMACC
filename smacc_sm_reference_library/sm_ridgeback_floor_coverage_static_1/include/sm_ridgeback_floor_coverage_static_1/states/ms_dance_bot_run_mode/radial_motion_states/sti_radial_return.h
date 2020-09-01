namespace sm_ridgeback_floor_coverage_static_1
{
  namespace radial_motion_states
  {
    // STATE DECLARATION
    struct StiRadialReturn : smacc::SmaccState<StiRadialReturn, SS>
    {
      using SmaccState::SmaccState;

      // TRANSITION TABLE
      typedef mpl::list<

          Transition<EvCbSuccess<CbUndoPathBackwards, OrNavigation>, StiRadialLoopStart, SUCCESS>,
          Transition<EvCbFailure<CbUndoPathBackwards, OrNavigation>, StiRadialEndPoint, ABORT>

          >
          reactions;

      // STATE FUNCTIONS
      static void staticConfigure()
      {
        configure_orthogonal<OrNavigation, CbUndoPathBackwards>();
        configure_orthogonal<OrLED, CbLEDOff>();
      }

      void runtimeConfigure()
      {
      }
    };
  } // namespace radial_motion_states
} // namespace sm_ridgeback_floor_coverage_static_1