#pragma once

#include <smacc/smacc_client.h>
#include <boost/optional/optional_io.hpp>

namespace smacc
{

class SmaccPublisherClient : public smacc::ISmaccClient
{
public:
  boost::optional<std::string> topicName;
  boost::optional<int> queueSize;

  SmaccPublisherClient()
  {
    initialized_ = false;
  }

  virtual ~SmaccPublisherClient()
  {
    pub_.shutdown();
  }

  template <typename MessageType>
  void configureMessageType()
  {
    ROS_INFO_STREAM("[" << this->getName() << "] Client Publisher to topic: " << topicName);
    pub_ = nh_.advertise<MessageType>(*topicName, *queueSize);
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

        this->initialized_ = true;
      }
    }
  }

  template <typename MessageType>
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