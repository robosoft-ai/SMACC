#pragma once

#include <smacc/client.h>

namespace smacc
{
template <typename MessageType>
class SmaccTopicPublisherClient : public smacc::ISmaccClient
{
public:
  SmaccTopicPublisherClient()
  {
    initialized_ = false;
  }

  virtual ~SmaccTopicPublisherClient()
  {
    pub_.shutdown();
  }

  void initialize(std::string topicName, int queueSize = 1)
  {
    if (!initialized_)
    {
      ROS_INFO_STREAM("[" << this->getName() << "] Client Publisher to topic: " << topicName);
      pub_ = nh_.advertise<MessageType>(topicName, queueSize);
      this->initialized_=true;
    }
  }

  void publish(const MessageType& msg)
  {
    pub_.publish(msg);
  }

protected:
  ros::NodeHandle nh_;
  ros::Publisher pub_;

private:
  bool initialized_;
};
}