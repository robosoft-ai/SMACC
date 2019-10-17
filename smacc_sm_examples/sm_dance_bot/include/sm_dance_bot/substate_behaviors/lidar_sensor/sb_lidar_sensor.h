#include <smacc_interface_components/substate_behaviors/sensor_substate.h>
#include <sensor_msgs/LaserScan.h>

class SbLidarSensor : public smacc::SensorTopic<SbLidarSensor, sensor_msgs::LaserScan>
{
public:
  virtual void onMessageCallback(const sensor_msgs::LaserScan &msg) override
  {
  }
};
