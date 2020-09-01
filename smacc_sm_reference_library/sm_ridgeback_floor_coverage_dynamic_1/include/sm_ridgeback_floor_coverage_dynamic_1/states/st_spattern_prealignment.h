#include <smacc/smacc.h>
namespace sm_ridgeback_floor_coverage_dynamic_1
{
  // STATE DECLARATION
  struct StSpatternPrealignment : smacc::SmaccState<StSpatternPrealignment, MsDanceBotRunMode>
  {
    using SmaccState::SmaccState;

    // TRANSITION TABLE
    typedef mpl::list<

        Transition<EvCbSuccess<CbAbsoluteRotate, OrNavigation>, SS5::SsSPattern1>,
        Transition<EvCbFailure<CbAbsoluteRotate, OrNavigation>, StNavigateToWaypointsX>

        >
        reactions;

    // STATE FUNCTIONS
    static void staticConfigure()
    {
      configure_orthogonal<OrNavigation, CbAbsoluteRotate>(60.5);
      configure_orthogonal<OrLED, CbLEDOff>();
    }

    void runtimeConfigure()
    {
    }
  };
} // namespace sm_ridgeback_floor_coverage_dynamic_1