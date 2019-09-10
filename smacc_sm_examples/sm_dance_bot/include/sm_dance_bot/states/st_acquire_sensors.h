using namespace smacc;

typedef smacc::SensorTopic<sensor_msgs::LaserScan> LidarSensor;

struct st_acquire_sensors : smacc::SmaccState<st_acquire_sensors, sm_dance_bot>
{
   using SmaccState::SmaccState;

   mpl::list<
       sc::custom_reaction<EvSensorMessage<smacc::SensorTopic<sensor_msgs::LaserScan>>>,
       sc::custom_reaction<EvSensorMessage<CustomConditionTemperatureSensor>>,

       sc::transition<EvStateFinish<st_acquire_sensors>, st_navigate_to_waypoints_x>>
       reactions;

   AllEventAggregator allSensorsReady;

   void onInitialize()
   {
      this->configure<ObstaclePerceptionOrthogonal>(new smacc::SensorTopic<sensor_msgs::LaserScan>("/front/scan", 1, ros::Duration(10)));
      this->configure<SensorOrthogonal>(new CustomConditionTemperatureSensor("/temperature"));

      allSensorsReady.setTriggerEventTypesCount(2);
   }

   sc::result react(const EvSensorMessage<smacc::SensorTopic<sensor_msgs::LaserScan>> &ev)
   {
      ROS_INFO("Lidar sensor is ready");
      if (allSensorsReady.notify<EvSensorMessage<smacc::SensorTopic<sensor_msgs::LaserScan>>>())
         this->throwFinishEvent();
   }

   sc::result react(const EvSensorMessage<CustomConditionTemperatureSensor> &ev)
   {
      ROS_INFO("Temperature sensor is ready");
      if (allSensorsReady.notify<EvSensorMessage<CustomConditionTemperatureSensor>>())
         this->throwFinishEvent();
   }
};
