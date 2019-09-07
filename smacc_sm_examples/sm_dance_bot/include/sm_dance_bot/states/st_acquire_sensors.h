
using namespace smacc;

typedef smacc::SensorTopic<sensor_msgs::LaserScan> LidarSensor;

struct st_acquire_sensors: smacc::SmaccState<st_acquire_sensors,sm_dance_bot>
{
   using SmaccState::SmaccState;

   mpl::list<
   
            sc::custom_reaction<EvSensorMessage<LidarSensor>>,
            sc::custom_reaction<EvSensorMessage<CustomConditionTemperatureSensor>>,

            sc::transition<EvStateFinish<st_acquire_sensors>, st_navigate_to_waypoints_x>
             > reactions; 

   AllEventAggregator allSensorsReady;

   void onInitialize()
   {
      this->configure<ObstaclePerceptionOrthogonal>(new LidarSensor("/scan", 1, ros::Duration(10)));
      this->configure<SensorOrthogonal>(new CustomConditionTemperatureSensor("/temperature"));

      allSensorsReady.setTriggerEventTypesCount(2);
   }

   sc::result react(const EvSensorMessage<LidarSensor> &ev) 
   {
      if(allSensorsReady.notify<EvSensorMessage<LidarSensor>>())
         this->throwFinishEvent();
   }

   sc::result react(const EvSensorMessage<CustomConditionTemperatureSensor> &ev) 
   {
      if(allSensorsReady.notify<EvSensorMessage<CustomConditionTemperatureSensor>>())
        this->throwFinishEvent();
   }
};
