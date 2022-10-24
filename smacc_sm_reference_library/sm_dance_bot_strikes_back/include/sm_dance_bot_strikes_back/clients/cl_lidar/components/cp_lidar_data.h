
#include <smacc/smacc.h>

namespace sm_dance_bot_strikes_back
{
namespace cl_lidar
{
class CpLidarSensorData : public smacc::ISmaccComponent
{
public:
  sensor_msgs::LaserScan lastMessage_;
  float forwardObstacleDistance;

  const float SECURITY_DISTANCE =  1; //meters

  virtual void onInitialize() override
  {
    auto client_ = dynamic_cast<smacc::client_bases::SmaccSubscriberClient<sensor_msgs::LaserScan> *>(owner_);
    client_->onMessageReceived(&CpLidarSensorData::MessageCallbackStoreDistanceToWall, this);
  }

  void MessageCallbackStoreDistanceToWall(const sensor_msgs::LaserScan &scanmsg)
  {
    this->lastMessage_ = scanmsg;
    auto fwdist = scanmsg.ranges[scanmsg.ranges.size() / 2] /*meter*/;

    /*if the distance is infinity or nan, use max range distance*/
    if (fwdist == std::numeric_limits<float>::infinity() || fwdist != fwdist)
    {
      this->forwardObstacleDistance = scanmsg.range_max - SECURITY_DISTANCE /*meters*/;
      ROS_INFO_THROTTLE(1.0, "[CpLidarSensorData] Distance to forward obstacle is not a number, setting default value to: %lf", scanmsg.range_max);
    }
    else
    {
      this->forwardObstacleDistance = std::max(fwdist - SECURITY_DISTANCE, 0.0F);
    }
  }
};
} // namespace cl_lidar
} // namespace sm_dance_bot_2
