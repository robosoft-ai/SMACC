#include <smacc_interface_components/substate_behaviors/sensor_substate.h>
#include <sensors/sensor_substate.h>
#include <sensor_msgs/Temperature.h>

struct CustomTemperatureAlertEvent: sc::event<CustomTemperatureAlertEvent>
{

};

//--------------------------------------------------------------------------------------
class CustomConditionTemperatureSensor: public smacc::SensorTopic<sensor_msgs::Temperature>
{
  using smacc::SensorTopic<sensor_msgs::Temperature>::InitialMessageEvent;
  using smacc::SensorTopic<sensor_msgs::Temperature>::MessageTimeoutEvent;
  using smacc::SensorTopic<sensor_msgs::Temperature>::MessageEvent;

  typedef CustomTemperatureAlertEvent TemperatureAlertEvent;

  public:
  CustomConditionTemperatureSensor(std::string topicName, int queueSize = 1, ros::Duration timeout= ros::Duration(5))
  : smacc::SensorTopic<sensor_msgs::Temperature>(topicName, queueSize, timeout)
  {
  }

  virtual void onMessageCallback(const sensor_msgs::Temperature& msg) override
  {
    if(msg.temperature > 40)
    {
      auto ev = new CustomTemperatureAlertEvent();
      this->postEvent(ev);
    }
  } 
};

