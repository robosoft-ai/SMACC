
#pragma once
#include <sensor_msgs/Temperature.h>
#include <sm_ridgeback_floor_coverage_static_1/clients/cl_temperature_sensor/cl_temperature_sensor.h>
#include <multirole_sensor_client/client_behaviors/cb_default_multirole_sensor_behavior.h>

namespace sm_ridgeback_floor_coverage_static_1
{
namespace cl_temperature_sensor
{
struct EvCustomTemperatureAlert : sc::event<EvCustomTemperatureAlert>
{
};

//--------------------------------------------------------------------------------------
class CbConditionTemperatureSensor : public cl_multirole_sensor::CbDefaultMultiRoleSensorBehavior<ClTemperatureSensor>
{
public:
  CbConditionTemperatureSensor()
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
} // namespace cl_temperature_sensor
} // namespace sm_ridgeback_floor_coverage_static_1
