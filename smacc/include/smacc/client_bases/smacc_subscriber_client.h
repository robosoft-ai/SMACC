/*****************************************************************************************************************
 * ReelRobotix Inc. - Software License Agreement      Copyright (c) 2018
 * 	 Authors: Pablo Inigo Blasco, Brett Aldrich
 *
 ******************************************************************************************************************/

#pragma once

#include <smacc/smacc_client.h>
#include <boost/optional/optional_io.hpp>
#include <smacc/impl/smacc_state_impl.h>

namespace smacc
{

namespace client_bases
{

using namespace smacc::default_events;

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

  SmaccSubscriberClient(std::string topicname)
  {
    topicName = topicname;
  }

  virtual ~SmaccSubscriberClient()
  {
    sub_.shutdown();
  }

  smacc::SmaccSignal<void(const MessageType &)> onFirstMessageReceived_;
  smacc::SmaccSignal<void(const MessageType &)> onMessageReceived_;

  std::function<void(const MessageType &)> postMessageEvent;
  std::function<void(const MessageType &)> postInitialMessageEvent;

  template <typename T>
  boost::signals2::connection onMessageReceived(void (T::*callback)(const MessageType &), T *object)
  {
    return this->getStateMachine()->createSignalConnection(onMessageReceived_, callback, object);
  }

  template <typename T>
  boost::signals2::connection onFirstMessageReceived(void (T::*callback)(const MessageType &), T *object)
  {
    return this->getStateMachine()->createSignalConnection(onFirstMessageReceived_, callback, object);
  }

  template <typename TOrthogonal, typename TSourceObject>
  void onOrthogonalAllocation()
  {
    this->postMessageEvent = [=](auto msg) {
      auto event = new EvTopicMessage<TSourceObject, TOrthogonal>();
      event->msgData = msg;
      this->postEvent(event);
    };

    this->postInitialMessageEvent = [=](auto msg) {
      auto event = new EvTopicInitialMessage<TSourceObject, TOrthogonal>();
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
      onFirstMessageReceived_(msg);
      firstMessage_ = false;
    }

    onMessageReceived_(msg);
    postMessageEvent(msg);
  }
};
} // namespace client_bases
} // namespace smacc
