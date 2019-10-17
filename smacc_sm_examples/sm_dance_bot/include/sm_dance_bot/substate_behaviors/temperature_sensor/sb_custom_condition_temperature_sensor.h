#include <smacc_interface_components/substate_behaviors/sensor_substate.h>
#include <sensor_msgs/Temperature.h>

struct EvCustomTemperatureAlert : sc::event<EvCustomTemperatureAlert>
{
};

//--------------------------------------------------------------------------------------
class SbConditionTemperatureSensor : public smacc::SensorTopic<SbConditionTemperatureSensor, sensor_msgs::Temperature>
{
public:
  virtual void onMessageCallback(const sensor_msgs::Temperature &msg) override
  {
    if (msg.temperature > 40)
    {
      auto ev = new EvCustomTemperatureAlert();
      this->postEvent(ev);
    }
  }
};
