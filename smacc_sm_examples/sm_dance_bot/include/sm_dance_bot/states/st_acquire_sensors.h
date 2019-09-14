
#include <boost/statechart/custom_reaction.hpp>

using namespace smacc;

struct StAcquireSensors : smacc::SmaccState<StAcquireSensors, SmDanceBot>
{
   using SmaccState::SmaccState;

   typedef mpl::list<
       sc::custom_reaction<EvSensorMessage<LidarSensor>>,
       sc::custom_reaction<EvSensorMessage<smacc::SensorTopic<sensor_msgs::Temperature>>>,

       sc::transition<EvStateFinish<StAcquireSensors>, StNavigateToWaypointsX>>
       reactions;

   AllEventAggregator allSensorsReady;

   void onInitialize()
   {
      this->configure<ObstaclePerceptionOrthogonal>(new LidarSensor("/front/scan", 1, ros::Duration(10)));
      this->configure<SensorOrthogonal>(new SbConditionTemperatureSensor("/temperature"));

      allSensorsReady.setTriggerEventTypesCount(2);
   }
   
   sc::result react(const EvSensorMessage<LidarSensor> &ev)
   {
      ROS_INFO_ONCE("Lidar sensor is ready");

      if (allSensorsReady.notify(ev))
         this->throwFinishEvent();

      return discard_event();
   }

   sc::result react(const EvSensorMessage<smacc::SensorTopic<sensor_msgs::Temperature>> &ev)
   {
      ROS_INFO_ONCE("Temperature sensor is ready");
      if (allSensorsReady.notify(ev))
         this->throwFinishEvent();

      return discard_event();
   }
};
