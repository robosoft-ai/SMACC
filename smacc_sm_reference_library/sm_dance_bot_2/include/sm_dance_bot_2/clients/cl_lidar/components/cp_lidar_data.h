
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
    auto client_ = dynamic_cast<smacc::SmaccSubscriberClient<sensor_msgs::LaserScan> *>(owner);
    client_->onMessageReceived(&CpLidarSensorData::storeMessage, this);
  }

  void storeMessage(const sensor_msgs::LaserScan &scanmsg)
  {
    this->lastMessage_ = scanmsg;
    this->forwardObstacleDistance = scanmsg.ranges[scanmsg.ranges.size() / 2];
  }
};
} // namespace cl_lidar
} // namespace sm_dance_bot_2