#include <smacc/smacc.h>
namespace sm_ridgeback_barrel_search_1
{
    using namespace smacc::default_events;
    // STATE DECLARATION
    struct StNavigateToWaypointX : smacc::SmaccState<StNavigateToWaypointX, SmRidgebackBarrelSearch1>
    {
        using SmaccState::SmaccState;

        // TRANSITION TABLE
        typedef mpl::list<

            Transition<EvCbSuccess<CbNavigateNextWaypoint, OrNavigation>, StDetectItems>,
            Transition<EvCbFailure<CbNavigateNextWaypoint, OrNavigation>, StNavigateToWaypointX>

            >
            reactions;

        // STATE FUNCTIONS
        static void staticConfigure()
        {
            // configure_orthogonal<OrLED, CbLEDOn>();
            // configure_orthogonal<OrObstaclePerception, CbLidarSensor>();
            configure_orthogonal<OrNavigation, CbNavigateNextWaypoint>();
        }

        void runtimeConfigure()
        {
        }
    };
} // namespace sm_ridgeback_barrel_search_1
