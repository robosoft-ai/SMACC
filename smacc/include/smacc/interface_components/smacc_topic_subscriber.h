#pragma once

#include <smacc/client.h>
#include <boost/signals2.hpp>

namespace smacc
{
template <typename SensorBehaviorType>
struct EvTopicInitialMessage : sc::event<EvTopicInitialMessage<SensorBehaviorType>>
{
  //typename EvTopicInitialMessage<SensorBehaviorType>::TMessageType msgData;
};

template <typename SensorBehaviorType>
struct EvTopicMessage : sc::event<EvTopicMessage<SensorBehaviorType>>
{
  //typename EvTopicInitialMessage<SensorBehaviorType>::TMessageType msgData;
};

template <typename MessageType>
class SmaccTopicSubscriberClient : public smacc::ISmaccClient
{
public:
  boost::signals2::signal<void(const MessageType &)> onFirstMessageReceived;
  boost::signals2::signal<void(const MessageType &)> onMessageReceived;

  SmaccTopicSubscriberClient()
  {
    initialized_ = false;
  }

  virtual ~SmaccTopicSubscriberClient()
  {
  }

  virtual void initialize(std::string topicName, int queueSize = 1)
  {
    if (!initialized_)
    {
      firstMessage_ = true;
      ROS_INFO_STREAM("[" << this->getName() << "]Subscribing to sensor topic: " << topicName);
      sub_ = nh_.subscribe(topicName, queueSize, &SmaccTopicSubscriberClient<MessageType>::messageCallback, this);
    }
  }

protected:
  ros::NodeHandle nh_;

private:
  ros::Subscriber sub_;
  bool firstMessage_;
  bool initialized_;

  void messageCallback(const MessageType &msg)
  {
    if (firstMessage_)
    {
      auto event = new EvTopicInitialMessage<SmaccTopicSubscriberClient<MessageType>>();
      this->postEvent(event);
      this->onFirstMessageReceived(msg);
      firstMessage_ = false;
    }

    auto *ev2 = new EvTopicMessage<SmaccTopicSubscriberClient<MessageType>>();
    this->postEvent(ev2);
    onMessageReceived(msg);
  }
};
}