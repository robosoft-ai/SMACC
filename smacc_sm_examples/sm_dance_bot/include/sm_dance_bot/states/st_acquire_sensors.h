using namespace smacc;


struct StAcquireSensors : smacc::SmaccState<StAcquireSensors, SmDanceBot>
{
   template<typename TSource>
   struct EvAll: sc::event<EvAll<TSource>>
   {
   };

   template<typename TSource>
   struct EvAll2: sc::event<EvAll2<TSource>>
   {
   };


   using SmaccState::SmaccState;

   typedef mpl::list<

       // Expected event
       //sc::transition<EvStateFinish<StAcquireSensors>, StNavigateToWaypointsX>,
       sc::transition<EvAll<LuAll>, StNavigateToWaypointsX>,

       // Keyboard event
       sc::transition<EvKeyPressN<SbKeyboard>, StNavigateToWaypointsX>//,
       
       //sc::transition<EvAll2<LuAl2>, StateDestiny2>,

       // Sensor events
       //sc::custom_reaction<EvTopicMessage<LidarSensor>>,
       //sc::custom_reaction<EvTopicMessage<smacc::SensorTopic<sensor_msgs::Temperature>>>>
       >
       reactions;

   //AllEventAggregator allSensorsReady;

   static void onDefinition()
   {
      static_configure<ObstaclePerceptionOrthogonal, LidarSensor>();
      static_configure<KeyboardOrthogonal, SbKeyboard>();
      static_configure<PublisherOrthogonal, SbStringPublisher>("Hello World!");
      static_configure<SensorOrthogonal, SbConditionTemperatureSensor>();
      static_configure<Service3Orthogonal, Service3Behavior>(Service3Command::SERVICE3_ON);

      static_createLogicUnit<LuAll, EvAll<LuAll>, EvTopicMessage<LidarSensor>, EvTopicMessage<smacc::SensorTopic<sensor_msgs::Temperature>>>();
      //static_createLogicUnit<LuAll2, EvAll2<LuAl2>,  EvKeyPressN<SbKeyboard>, EvKeyPressP<SbKeyboard>>();
   }

   void onInitialize()
   {
      //allSensorsReady.setTriggerEventTypesCount(2);
   }

/*
   sc::result react(const EvTopicMessage<LidarSensor> &ev)
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
