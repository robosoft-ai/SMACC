#include <smacc/smacc.h>
namespace sm_dance_bot_3
{
struct StEventCountDown : smacc::SmaccState<StEventCountDown, MsDanceBotRunMode>
{
    using SmaccState::SmaccState;

    typedef mpl::list<
        // Expected event
        Transition<EvCountdownEnd<SrEventCountdown>, StNavigateToWaypointsX>,

        Transition<EvGlobalError, sc::deep_history<StAcquireSensors>>>
        reactions;

    static void staticConfigure()
    {
        //   configure_orthogonal<OrObstaclePerception, CbLidarSensor>();
        //   configure_orthogonal<OrStringPublisher, CbStringPublisher>("Hello World!");
        //   configure_orthogonal<OrTemperatureSensor, CbConditionTemperatureSensor>();
        //   configure_orthogonal<OrService3, CbService3>(Service3Command::SERVICE3_ON);

        //static_createStateReactor<SrEventCountdown, EvCountdownEnd<SrEventCountdown>, mpl::list<EvActionFeedback<ClMoveBaseZ>>>(100);
        //static_createStateReactor<SrEventCountdown, EvCountdownEnd<SrEventCountdown>, mpl::list<EvTimer<ClRosTimer, OrTimer>>>(5);        
        
        auto srCountdown = static_createStateReactor<SrEventCountdown>(5);        
        srCountdown->addInputEvent<EvTimer<ClRosTimer, OrTimer>>();
        srCountdown->setOutputEvent<EvCountdownEnd<SrEventCountdown>>();
    }
};
}