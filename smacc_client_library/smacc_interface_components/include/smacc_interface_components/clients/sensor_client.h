#pragma once

#include <smacc/client_bases/smacc_subscriber_client.h>
#include <boost/statechart/event.hpp>

#include <ros/ros.h>
#include <ros/duration.h>
#include <boost/signals2.hpp>
#include <boost/optional/optional_io.hpp>

namespace smacc
{

template <typename TSource, typename TObjectTag>
struct EvTopicMessageTimeout : sc::event<EvTopicMessageTimeout<TSource, TObjectTag>>
{
  ros::TimerEvent timerData;
};

//---------------------------------------------------------------
template <typename MessageType>
class SensorClient : public SmaccSubscriberClient<MessageType>
{
public:
  typedef MessageType TMessageType;
  boost::signals2::signal<void(const MessageType &)> onMessageTimeout;

  SensorClient()
      : SmaccSubscriberClient<MessageType>()
  {
    ROS_INFO("CbLidarSensor constructor");
    initialized_ = false;
  }

  std::function<void(const ros::TimerEvent &ev)> postTimeoutMessageEvent;

  template <typename TDerived, typename TObjectTag>
  void configureEventSourceTypes()
  {
    SmaccSubscriberClient<MessageType>::template configureEventSourceTypes<TDerived, TObjectTag>();

    this->postTimeoutMessageEvent = [=](auto &timerdata) {
      auto event = new EvTopicMessageTimeout<TDerived, TObjectTag>();
      event->timerData = timerdata;
      this->postEvent(event);
    };
  }

  virtual void initialize() override
  {
    if (!initialized_)
    {
      SmaccSubscriberClient<MessageType>::initialize();

      this->onMessageReceived.connect(
          [this](auto msg) {
            //reseting the timer
            this->timeoutTimer_.stop();
            this->timeoutTimer_.start();
          });

      if (timeout_)
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

  void timeoutCallback(const ros::TimerEvent &timerdata)
  {
    postTimeoutMessageEvent(timerdata);
  }
};
} // namespace smacc