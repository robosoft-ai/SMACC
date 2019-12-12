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
      static_configure<ObstaclePerceptionOrthogonal, SbLidarSensor>();
      static_configure<PublisherOrthogonal, SbStringPublisher>("Hello World!");
      static_configure<SensorOrthogonal, SbConditionTemperatureSensor>();
      static_configure<Service3Orthogonal, Service3Behavior>(Service3Command::SERVICE3_ON);
      static_configure<UpdatablePublisherOrthogonal, SbUpdatableStringPublisher>();

      static_createLogicUnit<LuAllEventsGo,
                             EvAll<LuAllEventsGo, Unit1>,
                             mpl::list<EvTopicMessage<SbLidarSensor>, EvTopicMessage<SbConditionTemperatureSensor>>>();
   }

   void onInitialize()
   {
      SmaccMoveBaseActionClient *move_base_action_client;
      this->requiresClient(move_base_action_client);

      // we careful with the lifetime of the callbac, us a scoped connection if is not forever
      //move_base_action_client->onSucceeded(&StAcquireSensors::onActionClientSucceded, this);
   }

   void onActionClientSucceded(SmaccMoveBaseActionClient::ResultConstPtr &msg)
   {
      ROS_INFO_STREAM("Success Detected from StAquireSensors (connected to client signal), result data: " << *msg);
   }
};
} // namespace sm_dance_bot