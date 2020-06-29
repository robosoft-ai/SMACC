#include <smacc/smacc.h>
namespace sm_opencv_2
{
    using namespace smacc::default_events;
    // STATE DECLARATION
    struct StNavigateToWaypointX : smacc::SmaccState<StNavigateToWaypointX, SmOpenCV2>
    {
        using SmaccState::SmaccState;

        // TRANSITION TABLE
        typedef mpl::list<
            Transition<smacc::default_events::EvActionSucceeded<ClMoveBaseZ, OrNavigation>, StDetectItems>,
            Transition<smacc::default_events::EvActionAborted<ClMoveBaseZ, OrNavigation>, StNavigateToWaypointX>
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
} // namespace sm_opencv_2