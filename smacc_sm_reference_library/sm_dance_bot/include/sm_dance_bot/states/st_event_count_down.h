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
        //   static_configure<ObstaclePerceptionOrthogonal, CbLidarSensor>();
        //   static_configure<PublisherOrthogonal, CbStringPublisher>("Hello World!");
        //   static_configure<SensorOrthogonal, CbConditionTemperatureSensor>();
        //   static_configure<Service3Orthogonal, Service3Behavior>(Service3Command::SERVICE3_ON);

        static_createLogicUnit<LuEventCountdown, EvCountdownEnd<LuEventCountdown>, mpl::list<EvTimer<SmaccTimerClient>>>(5);

        //static_createLogicUnit<LuEventCountdown, EvCountdownEnd<LuEventCountdown>, mpl::list<EvActionFeedback<smacc::SmaccMoveBaseActionClient>>>(100);
    }
};
}