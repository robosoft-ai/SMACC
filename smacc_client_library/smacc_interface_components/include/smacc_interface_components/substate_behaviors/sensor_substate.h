#pragma once

#include <smacc/smacc_substate_behavior.h>
#include <boost/statechart/event.hpp>

#include <ros/ros.h>
#include <ros/duration.h>
#include <boost/signals2.hpp>

namespace smacc
{

//----------------- TIMER EVENT DEFINITION ----------------------------------------------
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

template <typename SensorBehaviorType>
struct EvTopicMessageTimeout : sc::event<EvTopicMessageTimeout<SensorBehaviorType>>
{
};

struct testEvent : sc::event<testEvent>
{
};
//---------------------------------------------------------------

template <typename MessageType>
class SmaccTopicSubcriber : public smacc::ISmaccComponent
{
public:
  boost::signals2::signal<void(const MessageType &)> onFirstMessageReceived;
  boost::signals2::signal<void(const MessageType &)> onMessageReceived;

  SmaccTopicSubcriber()
  {
    initialized_ = false;
  }

  virtual ~SmaccTopicSubcriber()
  {
  }

  void tryStart(std::string topicName, int queueSize = 1)
  {
    if (!initialized_)
    {
      firstMessage_ = true;
      ROS_INFO_STREAM("[" << this->getName() << "]Subscribing to sensor topic: " << topicName);
      sub_ = nh_.subscribe(topicName, queueSize, &SmaccTopicSubcriber<MessageType>::messageCallback, this);
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
      auto event = new EvTopicInitialMessage<SmaccTopicSubcriber<MessageType>>();
      this->postEvent(event);
      this->onFirstMessageReceived(msg);
      firstMessage_ = false;
    }

    auto *ev2 = new EvTopicMessage<SmaccTopicSubcriber<MessageType>>();
    this->postEvent(ev2);
    onMessageReceived(msg);
  }
};

//---------------------------------------------------------------
template <typename MessageType>
class SensorClient : public SmaccTopicSubcriber<MessageType>
{
public:
  boost::signals2::signal<void(const MessageType &)> onMessageTimeout;

  SensorClient()
      : SmaccTopicSubcriber<MessageType>()
  {
    initialized_ = false;
  }

  void tryStart(std::string topicName, int queueSize = 1, ros::Duration timeout = ros::Duration(5))
  {
    if (!initialized_)
    {
      SmaccTopicSubcriber<MessageType>::tryStart(topicName, queueSize);

      this->onMessageReceived.connect(
          [this](auto msg) {
            //reseting the timer
            this->timeoutTimer_.stop();
            this->timeoutTimer_.start();
          });

      timeoutTimer_ = this->nh_.createTimer(timeout, boost::bind(&SensorClient<MessageType>::timeoutCallback, this, _1));
      timeoutTimer_.start();
      initialized_ = true;
    }
  }

private:
  ros::Timer timeoutTimer_;
  bool initialized_;

  void timeoutCallback(const ros::TimerEvent &ev)
  {
    auto event = new EvTopicMessageTimeout<SensorClient<MessageType>>();
    this->postEvent(event);
  }
};

//------------------  TIMER SUBSTATE ---------------------------------------------

template <typename MessageType>
class SensorTopic : public smacc::SmaccSubStateBehavior
{
public:
  typedef MessageType TMessageType;

  SensorClient<MessageType> *sensor_;

  ros::Duration timeoutDuration_;
  std::string topicName_;
  int queueSize_;

  SensorTopic(std::string topicName, int queueSize = 1, ros::Duration timeout = ros::Duration(5))
  {
    timeoutDuration_ = timeout;
    queueSize_ = queueSize;
    topicName_ = topicName;
  }

  void onEntry()
  {
    this->requiresComponent(sensor_);
    sensor_->onMessageReceived.connect(
        [this](auto &msg) {
          auto *ev2 = new EvTopicMessage<SensorTopic<MessageType>>();
          this->postEvent(ev2);
        });

    sensor_->onFirstMessageReceived.connect(
        [this](auto &msg) {
          auto event = new EvTopicInitialMessage<SensorTopic<MessageType>>();
          this->postEvent(event);
        });

    sensor_->onMessageTimeout.connect(
        [this](auto &msg) {
          auto event = new EvTopicMessageTimeout<SensorTopic<MessageType>>();
          this->postEvent(event);
        });

    sensor_->tryStart(topicName_, queueSize_, timeoutDuration_);
  }

  bool onExit()
  {
  }

  virtual void onMessageCallback(const MessageType &msg)
  {

    // empty to fill by sensor customization based on inheritance
  }
};
} // namespace smacc
