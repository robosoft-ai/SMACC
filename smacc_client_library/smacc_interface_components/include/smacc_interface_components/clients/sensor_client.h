#pragma once

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
}