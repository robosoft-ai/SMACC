#include <smacc/smacc.h>
namespace sm_ridgeback_floor_coverage_static_1
{
   // STATE DECLARATION
   struct StNavigateReverse2 : smacc::SmaccState<StNavigateReverse2, MsDanceBotRunMode>
   {
      using SmaccState::SmaccState;

      // TRANSITION TABLE
      typedef mpl::list<

          Transition<EvCbSuccess<CbNavigateBackwards, OrNavigation>, StNavigateToWaypointsX>,
          Transition<EvCbFailure<CbNavigateBackwards, OrNavigation>, StNavigateToWaypointsX>

          >
          reactions;

      // STATE FUNCTIONS
      static void staticConfigure()
      {
         configure_orthogonal<OrNavigation, CbNavigateBackwards>(2);
         configure_orthogonal<OrLED, CbLEDOff>();
         configure_orthogonal<OrObstaclePerception, CbLidarSensor>();
      }
   };
} // namespace sm_ridgeback_floor_coverage_static_1