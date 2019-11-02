using namespace smacc;


// ---- TAGS ----
struct ON_KEYBOARD{};
struct ON_SENSORS_AVAILABLE{};
struct Unit1;
//----------------

struct StAcquireSensors : smacc::SmaccState<StAcquireSensors, SmDanceBot>
{
   using SmaccState::SmaccState;

   typedef mpl::list<

       // Expected event
       transition<EvAll<LuAllEventsGo, Unit1>, StNavigateToWaypointsX, ON_SENSORS_AVAILABLE>,

       // Keyboard event
       transition<EvKeyPressN<SbKeyboard>, StNavigateToWaypointsX, ON_KEYBOARD> 

       //smacc::transition<EvAll2<LuAl2>, StateDestiny2>,
       >
       reactions;

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
