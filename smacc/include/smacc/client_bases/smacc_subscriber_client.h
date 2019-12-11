#pragma once

#include <smacc/smacc_client.h>
#include <boost/signals2.hpp>
#include <boost/optional/optional_io.hpp>

namespace smacc
{
template <typename TSource>
struct EvTopicInitialMessage : sc::event<EvTopicInitialMessage<TSource>>
{
  //typename EvTopicInitialMessage<SensorBehaviorType>::TMessageType msgData;
  static std::string getEventLabel()
  {
    auto typeinfo = TypeInfo::getTypeInfoFromType<typename TSource::TMessageType>();

    std::string label = typeinfo->getNonTemplatetypename();
    return label;
  }
};

template <typename TSource>
struct EvTopicMessage : sc::event<EvTopicMessage<TSource>>
{
  static std::string getEventLabel()
  {
    auto typeinfo = TypeInfo::getTypeInfoFromType<typename TSource::TMessageType>();

    std::string label = typeinfo->getNonTemplatetypename();
    return label;
  }

  typename TSource::TMessageType msgData;
};

template <typename TDerived, typename MessageType>
class SmaccSubscriberClient : public smacc::ISmaccClient
{
public:
  boost::signals2::signal<void(const MessageType &)> onFirstMessageReceived;
  boost::signals2::signal<void(const MessageType &)> onMessageReceived;

  boost::optional<std::string> topicName;
  boost::optional<int> queueSize;

  typedef MessageType TMessageType;

  SmaccSubscriberClient()
  {
    initialized_ = false;
  }

  virtual ~SmaccSubscriberClient()
  {
    sub_.shutdown();
  }

  virtual void initialize()
  {
    if (!initialized_)
    {
      firstMessage_ = true;

      if (!queueSize)
        queueSize = 1;

      if (!topicName)
      {
        ROS_ERROR("topic client with no topic name set. Skipping subscribing");
      }
      else
      {
        ROS_INFO_STREAM("[" << this->getName() << "] Subscribing to topic: " << topicName);

        sub_ = nh_.subscribe(*topicName, *queueSize, &SmaccSubscriberClient<TDerived, MessageType>::messageCallback, this);
        this->initialized_ = true;
      }
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
      auto event = new EvTopicInitialMessage<TDerived>();
      this->postEvent(event);
      this->onFirstMessageReceived(msg);
      firstMessage_ = false;
    }

    auto *ev2 = new EvTopicMessage<TDerived>();
    this->postEvent(ev2);
    onMessageReceived(msg);
  }
};
} // namespace smacc