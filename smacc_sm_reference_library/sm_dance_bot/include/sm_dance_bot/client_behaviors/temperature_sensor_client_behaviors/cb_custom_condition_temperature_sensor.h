
#pragma once
#include <sensor_msgs/Temperature.h>
#include <sm_dance_bot/clients/cl_temperature_sensor.h>
#include <smacc_interface_components/client_behaviors/cb_sensor_base.h>

namespace sm_dance_bot
{
struct EvCustomTemperatureAlert : sc::event<EvCustomTemperatureAlert>
{
};

//--------------------------------------------------------------------------------------
class CbConditionClTemperatureSensor : public smacc::SensorTopic<ClTemperatureSensor>
{
public:
  CbConditionClTemperatureSensor()
  {
  }
  virtual void onMessageCallback(const sensor_msgs::Temperature &msg) override
  {
    if (msg.temperature > 40)
    {
      auto ev = new EvCustomTemperatureAlert();
      this->postEvent(ev);
    }
  }
};
} // namespace sm_dance_bot
