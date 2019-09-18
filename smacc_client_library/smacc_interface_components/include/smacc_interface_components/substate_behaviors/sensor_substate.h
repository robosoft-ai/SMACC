#pragma once

#include <smacc/smacc_substate_behavior.h>
#include <smacc/interface_components/smacc_topic_subscriber.h>
#include <boost/statechart/event.hpp>

#include <ros/ros.h>
#include <ros/duration.h>
#include <boost/signals2.hpp>

namespace smacc
{

template <typename SensorBehaviorType>
struct EvTopicMessageTimeout : sc::event<EvTopicMessageTimeout<SensorBehaviorType>>
{
};

//---------------------------------------------------------------
template <typename MessageType>
class SensorClient : public SmaccTopicSubscriberClient<MessageType>
{
public:
  boost::signals2::signal<void(const MessageType &)> onMessageTimeout;

  SensorClient()
      : SmaccTopicSubscriberClient<MessageType>()
  {
    initialized_ = false;
  }

  virtual void initialize(std::string topicName, int queueSize) override
  {
    if (!initialized_)
    {
      SmaccTopicSubscriberClient<MessageType>::initialize(topicName, queueSize);

      this->onMessageReceived.connect(
          [this](auto msg) {
            //reseting the timer
            this->timeoutTimer_.stop();
            this->timeoutTimer_.start();
          });

      timeoutTimer_ = this->nh_.createTimer(timeout_, boost::bind(&SensorClient<MessageType>::timeoutCallback, this, _1));
      timeoutTimer_.start();
      initialized_ = true;
    }
  }

  void initialize(std::string topicName, int queueSize, ros::Duration timeout)
  {
    this->timeout_ = timeout;
    this->initialize(topicName, queueSize);
  }

private:
  ros::Timer timeoutTimer_;
  bool initialized_;
  ros::Duration timeout_;

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

    sensor_->initialize(topicName_, queueSize_, timeoutDuration_);
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
