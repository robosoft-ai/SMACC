#pragma once

#include <smacc/interface_components/smacc_topic_subscriber.h>
#include <boost/statechart/event.hpp>

#include <ros/ros.h>
#include <ros/duration.h>
#include <boost/signals2.hpp>
#include <boost/optional/optional_io.hpp>

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

  virtual void initialize() override
  {
    if (!initialized_)
    {
      SmaccTopicSubscriberClient<MessageType>::initialize();

      this->onMessageReceived.connect(
          [this](auto msg) {
            //reseting the timer
            this->timeoutTimer_.stop();
            this->timeoutTimer_.start();
          });

      if(timeout_)
      {
        timeoutTimer_ = this->nh_.createTimer(*timeout_, boost::bind(&SensorClient<MessageType>::timeoutCallback, this, _1));
        timeoutTimer_.start();
      }
      else
      {
        ROS_WARN("Timeout sensor client not set, skipping timeout watchdog funcionality");
      }
      
      initialized_ = true;
    }
  }

  boost::optional<ros::Duration> timeout_;

private:
  ros::Timer timeoutTimer_;
  bool initialized_;
  
  void timeoutCallback(const ros::TimerEvent &ev)
  {
    auto event = new EvTopicMessageTimeout<SensorClient<MessageType>>();
    this->postEvent(event);
  }
};
}