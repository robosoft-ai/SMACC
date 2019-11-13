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

   typedef mpl::list <
         // Expected event
         transition<EvAll<LuAllEventsGo, Unit1>, StNavigateToWaypointsX, ON_SENSORS_AVAILABLE>,

         //smacc::transition<EvAll2<LuAl2>, StateDestiny2>,
         smacc::transition<EvGlobalError, sc::deep_history<StAcquireSensors>> 
      > reactions;

   //AllEventAggregator allSensorsReady;

   static void onDefinition()
   {
      static_configure<ObstaclePerceptionOrthogonal, SbLidarSensor>();
      static_configure<KeyboardOrthogonal, SbKeyboard>();
      static_configure<PublisherOrthogonal, SbStringPublisher>("Hello World!");
      static_configure<SensorOrthogonal, SbConditionTemperatureSensor>();
      static_configure<Service3Orthogonal, Service3Behavior>(Service3Command::SERVICE3_ON);

      static_createLogicUnit<LuAllEventsGo, EvAll<LuAllEventsGo, Unit1>, EvTopicMessage<SbLidarSensor>, EvTopicMessage<SbConditionTemperatureSensor>>();
   }

   void onInitialize()
   {
      //allSensorsReady.setTriggerEventTypesCount(2);
   }
};
