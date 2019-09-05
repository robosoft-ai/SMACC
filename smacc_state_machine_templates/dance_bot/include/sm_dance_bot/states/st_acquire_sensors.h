
using namespace smacc;

typedef smacc::SensorTopic<sensor_msgs::LaserScan> LidarSensor;

struct st_acquire_sensors: smacc::SmaccState<st_acquire_sensors,sm_dance_bot>
{
   using SmaccState::SmaccState;

   mpl::list<sc::custom_reaction<LidarSensor::MessageEvent>,
             sc::custom_reaction<CustomConditionTemperatureSensor::MessageEvent>,

             sc::transition<EvFinish, st_navigate_to_waypoints_x>
             > reactions; 


   AllEventAggregator allSensorsReady;

   void onInitialize()
   {
      this->configure<ObstaclePerceptionOrthogonal>(new LidarSensor("/scan", 1, ros::Duration(10)));
      this->configure<SensorOrthogonal>(new CustomConditionTemperatureSensor("/temperature"));

      allSensorsReady.setTriggerEventTypesCount(2);
   }

   sc::result react(const LidarSensor::MessageEvent &ev) 
   {
      if(allSensorsReady.notify<LidarSensor::MessageEvent>())
         this->throwFinishEvent();
   }

   sc::result react(const CustomConditionTemperatureSensor::MessageEvent &ev) 
   {
      if(allSensorsReady.notify<CustomConditionTemperatureSensor::MessageEvent>())
        this->throwFinishEvent();
   }
};
