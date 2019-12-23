#pragma once

#include <smacc/smacc_client.h>
#include <boost/signals2.hpp>
#include <boost/optional/optional_io.hpp>

namespace smacc
{
template <typename TSource, typename TObjectTag>
struct EvTopicInitialMessage : sc::event<EvTopicInitialMessage<TSource, TObjectTag>>
{
  //typename EvTopicInitialMessage<SensorBehaviorType>::TMessageType msgData;
  static std::string getEventLabel()
  {
    auto typeinfo = TypeInfo::getTypeInfoFromType<typename TSource::TMessageType>();

    std::string label = typeinfo->getNonTemplatetypename();
    return label;
  }

  typename TSource::TMessageType msgData;
};

template <typename TSource, typename TObjectTag>
struct EvTopicMessage : sc::event<EvTopicMessage<TSource, TObjectTag>>
{
  static std::string getEventLabel()
  {
    auto typeinfo = TypeInfo::getTypeInfoFromType<typename TSource::TMessageType>();

    std::string label = typeinfo->getNonTemplatetypename();
    return label;
  }

  typename TSource::TMessageType msgData;
};

template <typename MessageType>
class SmaccSubscriberClient : public smacc::ISmaccClient
{
public:
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

  boost::signals2::signal<void(const MessageType &)> onFirstMessageReceived;
  boost::signals2::signal<void(const MessageType &)> onMessageReceived;

  std::function<void(const MessageType &)> postMessageEvent;
  std::function<void(const MessageType &)> postInitialMessageEvent;

  template <typename TObjectTag, typename TDerived>
  void configureEventSourceTypes()
  {
    this->postMessageEvent = [=](auto msg) {
      auto event = new EvTopicMessage<TDerived, TObjectTag>();
      event->msgData = msg;
      this->postEvent(event);
    };

    this->postInitialMessageEvent = [=](auto msg) {
      auto event = new EvTopicInitialMessage<TDerived, TObjectTag>();
      event->msgData = msg;
      this->postEvent(event);
    };
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

        sub_ = nh_.subscribe(*topicName, *queueSize, &SmaccSubscriberClient<MessageType>::messageCallback, this);
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
      postInitialMessageEvent(msg);
      this->onFirstMessageReceived(msg);
      firstMessage_ = false;
    }

    postMessageEvent(msg);
    onMessageReceived(msg);
  }
};
} // namespace smacc