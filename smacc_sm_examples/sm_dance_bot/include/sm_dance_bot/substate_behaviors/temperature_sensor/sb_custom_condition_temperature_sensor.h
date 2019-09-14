#include <smacc_interface_components/substate_behaviors/sensor_substate.h>
#include <sensor_msgs/Temperature.h>

struct EvCustomTemperatureAlert: sc::event<EvCustomTemperatureAlert>
{

};

//--------------------------------------------------------------------------------------
class SbConditionTemperatureSensor: public smacc::SensorTopic<sensor_msgs::Temperature>
{
  public:
  
  typedef smacc::SensorTopic<sensor_msgs::Temperature> Base;
  
  SbConditionTemperatureSensor(std::string topicName, int queueSize = 1, ros::Duration timeout= ros::Duration(5))
  : smacc::SensorTopic<sensor_msgs::Temperature>(topicName, queueSize, timeout)
  {
  }

  virtual void onMessageCallback(const sensor_msgs::Temperature& msg) override
  {
    if(msg.temperature > 40)
    {
      auto ev = new EvCustomTemperatureAlert();
      this->postEvent(ev);
    }
  } 
};

