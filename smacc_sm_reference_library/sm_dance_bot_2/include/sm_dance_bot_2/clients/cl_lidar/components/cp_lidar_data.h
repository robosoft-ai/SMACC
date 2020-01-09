
#include <smacc/smacc.h>

namespace sm_dance_bot_2
{
namespace cl_lidar
{
class CpLidarSensorData : public smacc::ISmaccComponent
{
public:
  sensor_msgs::LaserScan lastMessage_;
  float forwardObstacleDistance;

  virtual void initialize(smacc::ISmaccClient *owner) override
  {
    auto client_ = dynamic_cast<multirole_sensor_client::SmaccSubscriberClient<sensor_msgs::LaserScan> *>(owner);
    client_->onMessageReceived(&CpLidarSensorData::MessageCallbackStoreDistanceToWall, this);
  }

  void MessageCallbackStoreDistanceToWall(const sensor_msgs::LaserScan &scanmsg)
  {
    this->lastMessage_ = scanmsg;
    auto fwdist = scanmsg.ranges[scanmsg.ranges.size() / 2] /*meter*/;

    float security_distance = 1.2; //meters

    /*if the distance is infinity or nan, use max range distance*/
    if (fwdist == std::numeric_limits<float>::infinity() || fwdist != fwdist)
    {
      this->forwardObstacleDistance = scanmsg.range_max - security_distance /*meters*/;
      ROS_INFO_THROTTLE(1.0, "[CpLidarSensorData] Distance to forward obstacle is not a number, setting default value to: %lf", scanmsg.range_max);
    }
    else
    {
      this->forwardObstacleDistance = fwdist - security_distance;
    }
  }
};
} // namespace cl_lidar
} // namespace sm_dance_bot_2