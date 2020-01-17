#include <smacc/smacc.h>
namespace sm_dance_bot
{
struct StEventCountDown : smacc::SmaccState<StEventCountDown, MsDanceBotRunMode>
{
    using SmaccState::SmaccState;

    typedef mpl::list<
        // Expected event
        Transition<EvCountdownEnd<SbEventCountdown>, StNavigateToWaypointsX>,

        Transition<EvGlobalError, sc::deep_history<StAcquireSensors>>>
        reactions;

    static void onDefinition()
    {
        //   configure_orthogonal<OrObstaclePerception, CbLidarSensor>();
        //   configure_orthogonal<OrStringPublisher, CbStringPublisher>("Hello World!");
        //   configure_orthogonal<OrTemperatureSensor, CbConditionTemperatureSensor>();
        //   configure_orthogonal<OrService3, CbService3>(Service3Command::SERVICE3_ON);

        static_createStateBehavior<SbEventCountdown, EvCountdownEnd<SbEventCountdown>, mpl::list<EvTimer<ClRosTimer, OrTimer>>>(5);

        //static_createStateBehavior<SbEventCountdown, EvCountdownEnd<SbEventCountdown>, mpl::list<EvActionFeedback<ClMoveBaseZ>>>(100);
    }
};
}