#pragma once

#include <smacc/client.h>
#include <boost/optional/optional_io.hpp>

namespace smacc
{
template <typename MessageType>
class SmaccTopicPublisherClient : public smacc::ISmaccClient
{
public:
  boost::optional<std::string> topicName;
  boost::optional<int> queueSize;

  SmaccTopicPublisherClient()
  {
    initialized_ = false;
  }

  virtual ~SmaccTopicPublisherClient()
  {
    pub_.shutdown();
  }

  virtual void initialize() override
  {
    if (!initialized_)
    {
      if (!queueSize)
        queueSize = 1;

      if (!topicName)
      {
        ROS_ERROR("topic publisher with no topic name set. Skipping advertising.");
      }
      else
      {
        ROS_INFO_STREAM("[" << this->getName() << "] Client Publisher to topic: " << topicName);
        pub_ = nh_.advertise<MessageType>(*topicName, *queueSize);
        this->initialized_ = true;
      }
    }
  }

  void publish(const MessageType &msg)
  {
    pub_.publish(msg);
  }

protected:
  ros::NodeHandle nh_;
  ros::Publisher pub_;

private:
  bool initialized_;
};
} // namespace smacc