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
       smacc::transition<EvAll<LuAll, Unit1>, StNavigateToWaypointsX, ON_SENSORS_AVAILABLE>,

       // Keyboard event
       smacc::transition<EvKeyPressN<SbKeyboard>, StNavigateToWaypointsX, ON_KEYBOARD> 

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

      static_createLogicUnit<LuAll, EvAll<LuAll, Unit1>, EvTopicMessage<SbLidarSensor>, EvTopicMessage<SbConditionTemperatureSensor>>();
   }

   void onInitialize()
   {
      //allSensorsReady.setTriggerEventTypesCount(2);
   }

   /*
   sc::result react(const EvTopicMessage<SbLidarSensor> &ev)
   {
      ROS_INFO_ONCE("Lidar sensor is ready");

      if (allSensorsReady.notify(ev))
         this->throwFinishEvent();

      return discard_event();
   }

   sc::result react(const EvTopicMessage<smacc::SensorTopic<sensor_msgs::Temperature>> &ev)
   {
      ROS_INFO_ONCE("Temperature sensor is ready");
      if (allSensorsReady.notify(ev))
         this->throwFinishEvent();

      return discard_event();
   }
   */
};
