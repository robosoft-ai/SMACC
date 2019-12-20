#include <smacc/smacc.h>
namespace sm_dance_bot
{
struct StAcquireSensors : smacc::SmaccState<StAcquireSensors, MsDanceBotRunMode>
{
   // transition names
   // ---- TAGS ----
   struct ON_KEYBOARD : PREEMPT
   {
   };

   struct ON_KEYBOARD2 : ABORT
   {
   };

   struct ON_SENSORS_AVAILABLE : SUCCESS
   {
   };

   struct Unit1;
   //----------------
   using SmaccState::SmaccState;

   typedef mpl::list<
       // Expected event
       //transition<EvAll<LuAllEventsGo, Unit1>, StNavigateToWaypointsX, ON_SENSORS_AVAILABLE>,
       transition<EvAll<LuAllEventsGo, Unit1>, StEventCountDown, ON_SENSORS_AVAILABLE>,

       //smacc::transition<EvAll2<LuAl2>, StateDestiny2>,
       smacc::transition<EvGlobalError, sc::deep_history<StAcquireSensors>>>
       reactions;

   static void onDefinition()
   {
      static_configure<OrObstaclePerception, CbLidarSensor>();
      static_configure<OrStringPublisher, CbStringPublisher>("Hello World!");
      static_configure<OrTemperatureSensor, CbConditionTemperatureSensor>();
      static_configure<OrService3, Service3Behavior>(Service3Command::SERVICE3_ON);
      static_configure<OrUpdatablePublisher, CbUpdatableStringPublisher>();

      static_createLogicUnit<LuAllEventsGo,
                             EvAll<LuAllEventsGo, Unit1>,
                             mpl::list<EvTopicMessage<CbLidarSensor, OrObstaclePerception>, EvTopicMessage<CbConditionTemperatureSensor, OrTemperatureSensor>>>();

      // auto luall = static_createLogicUnit<LuAllEventsGo>();
      // luall->enablesOn<EvTopicMessage<CbLidarSensor, OrObstaclePerception>, EvTopicMessage<CbConditionTemperatureSensor, OrTemperatureSensor>>();
      // luall->throwsEvent<EvAll<LuAllEventsGo, Unit1>>();
   }

   void onInitialize()
   {
      
   }
};
} // namespace sm_dance_bot