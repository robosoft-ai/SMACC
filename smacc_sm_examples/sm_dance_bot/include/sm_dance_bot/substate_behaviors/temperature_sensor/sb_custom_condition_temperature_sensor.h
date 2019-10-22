
#include <sensor_msgs/Temperature.h>
#include <sm_dance_bot/substate_behaviors/temperature_sensor/temperature_sensor.h>
#include <smacc_interface_components/substate_behaviors/sensor_substate.h>

namespace dance_bot
{
struct EvCustomTemperatureAlert : sc::event<EvCustomTemperatureAlert>
{
};

//--------------------------------------------------------------------------------------
class SbConditionTemperatureSensor : public smacc::SensorTopic<SbConditionTemperatureSensor, sensor_msgs::Temperature, TemperatureSensor>
{
public:
  SbConditionTemperatureSensor()
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
