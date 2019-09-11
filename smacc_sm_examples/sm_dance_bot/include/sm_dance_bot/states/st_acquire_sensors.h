
#include <boost/statechart/custom_reaction.hpp>

using namespace smacc;

typedef smacc::SensorTopic<sensor_msgs::LaserScan> LidarSensor;
typedef smacc::SensorTopic<sensor_msgs::Temperature> TemperatureSensor;

struct st_acquire_sensors : smacc::SmaccState<st_acquire_sensors, sm_dance_bot>
{
   using SmaccState::SmaccState;

   typedef mpl::list<
       sc::custom_reaction<EvSensorMessage<LidarSensor>>,
       sc::custom_reaction<EvSensorMessage<TemperatureSensor>>,

       sc::transition<EvStateFinish<st_acquire_sensors>, st_navigate_to_waypoints_x>>
       reactions;

   AllEventAggregator allSensorsReady;

   void onInitialize()
   {
      this->configure<ObstaclePerceptionOrthogonal>(new LidarSensor("/front/scan", 1, ros::Duration(10)));
      this->configure<SensorOrthogonal>(new TemperatureSensor("/temperature"));

      allSensorsReady.setTriggerEventTypesCount(2);
   }
   
   sc::result react(const EvSensorMessage<LidarSensor> &ev)
   {
      ROS_INFO_ONCE("Lidar sensor is ready");

      if (allSensorsReady.notify(ev))
         this->throwFinishEvent();

      return discard_event();
   }

   sc::result react(const EvSensorMessage<TemperatureSensor> &ev)
   {
      ROS_INFO_ONCE("Temperature sensor is ready");
      if (allSensorsReady.notify(ev))
         this->throwFinishEvent();

      return discard_event();
   }
};
