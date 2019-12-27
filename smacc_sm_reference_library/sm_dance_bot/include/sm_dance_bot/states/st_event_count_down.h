#include <smacc/smacc.h>
namespace sm_dance_bot
{
struct StEventCountDown : smacc::SmaccState<StEventCountDown, MsDanceBotRunMode>
{
    using SmaccState::SmaccState;

    typedef mpl::list<
        // Expected event
        transition<EvCountdownEnd<LuEventCountdown>, StNavigateToWaypointsX>,

        smacc::transition<EvGlobalError, sc::deep_history<StAcquireSensors>>>
        reactions;

    static void onDefinition()
    {
        //   static_configure<OrObstaclePerception, CbLidarSensor>();
        //   static_configure<OrStringPublisher, CbStringPublisher>("Hello World!");
        //   static_configure<OrTemperatureSensor, CbConditionTemperatureSensor>();
        //   static_configure<OrService3, CbService3>(Service3Command::SERVICE3_ON);

        static_createLogicUnit<LuEventCountdown, EvCountdownEnd<LuEventCountdown>, mpl::list<EvTimer<ClRosTimer, OrTimer>>>(5);

        //static_createLogicUnit<LuEventCountdown, EvCountdownEnd<LuEventCountdown>, mpl::list<EvActionFeedback<smacc::ClMoveBaseZ>>>(100);
    }
};
}