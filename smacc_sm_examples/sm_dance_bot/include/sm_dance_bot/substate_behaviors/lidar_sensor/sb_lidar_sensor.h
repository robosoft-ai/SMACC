#include <smacc_interface_components/substate_behaviors/sensor_substate.h>
#include <sensor_msgs/LaserScan.h>
#include <sm_dance_bot/substate_behaviors/lidar_sensor/lidar_client.h>

namespace dance_bot
{
struct SbLidarSensor : smacc::SensorTopic<SbLidarSensor, sensor_msgs::LaserScan, LaserSensor>
{

public:
  SbLidarSensor()
  {
    ROS_INFO("SbLidarSensor Constructor");
  }

  virtual void onEntry() override
  {
    ROS_INFO("SbLidarSensor onEntry");
    smacc::SensorTopic<SbLidarSensor, sensor_msgs::LaserScan, LaserSensor>::onEntry();
  }

  virtual void onMessageCallback(const sensor_msgs::LaserScan &msg) override
  {
  }
};
}
