#pragma once

#include <smacc/smacc_substate_behavior.h>
#include <boost/statechart/event.hpp>
#include <boost/signals2.hpp>

#include <ros/ros.h>
#include <ros/duration.h>

namespace smacc
{

//----------------- TIMER EVENT DEFINITION ----------------------------------------------
template <typename SensorBehaviorType>
struct EvSensorInitialMessage : sc::event<EvSensorInitialMessage<SensorBehaviorType>>
{
  //typename EvSensorInitialMessage<SensorBehaviorType>::TMessageType msgData;
};

template <typename SensorBehaviorType>
struct EvSensorMessage : sc::event<EvSensorMessage<SensorBehaviorType>>
{
  //typename EvSensorInitialMessage<SensorBehaviorType>::TMessageType msgData;
};

template <typename SensorBehaviorType>
struct EvSensorMessageTimeout : sc::event<EvSensorMessageTimeout<SensorBehaviorType>>
{
};

struct testEvent : sc::event<testEvent>
{
};

//------------------  SENSOR COMPONENT ---------------------------------------------

template <typename MessageType>
class SensorComponent : public smacc::ISmaccComponent
{
public:
  typedef MessageType TMessageType;

  ros::NodeHandle nh_;
  std::string topicName_;
  int queueSize_;
  ros::Subscriber sub_;
  bool firstTime_;
  ros::Timer timeoutTimer_;
  ros::Duration timeoutDuration_;
  bool started_;

  SensorComponent()
  {
    started_ = false;
    firstTime_ = true;
  }

  void tryStart(std::string topicName, int queueSize = 1, ros::Duration timeout = ros::Duration(5))
  {
    topicName_ = topicName;
    queueSize_ = queueSize;
    timeoutDuration_ = timeout;

    if (!started_)
    {
      ROS_INFO_STREAM("Subscribing to sensor topic: " << topicName_);
      sub_ = nh_.subscribe(topicName_, queueSize_, &SensorComponent<MessageType>::messageCallback, this);
      timeoutTimer_ = nh_.createTimer(this->timeoutDuration_, boost::bind(&SensorComponent<MessageType>::timeoutCallback, this, _1));
      timeoutTimer_.start();
      started_ = true;
    }
  }

  virtual ~SensorComponent()
  {
  }

  void timeoutCallback(const ros::TimerEvent &ev)
  {
    auto event = new EvSensorMessageTimeout<SensorComponent<MessageType>>();
    this->postEvent(event);

    OnSensorTimeout();
  }

  virtual void onMessageCallback(const MessageType &msg)
  {

    // empty to fill by sensor customization based on inheritance
  }

  // observer pattern
  boost::signals2::signal<void(const MessageType &)> OnSensorInitialMessage;
  boost::signals2::signal<void(const MessageType &)> OnSensorMessage;
  boost::signals2::signal<void()> OnSensorTimeout;

  void messageCallback(const MessageType &msg)
  {
    //ROS_INFO_STREAM("message received from: "<< this->topicName_);

    if (firstTime_)
    {
      firstTime_ = false;
      auto event = new EvSensorInitialMessage<SensorComponent<MessageType>>();
      this->postEvent(event);

      OnSensorInitialMessage(msg);
    }

    OnSensorMessage(msg);
    EvSensorMessage<SensorComponent<MessageType>> *ev2 = new EvSensorMessage<SensorComponent<MessageType>>();
    this->postEvent(ev2);

    this->onMessageCallback(msg);

    // reseting the timeout timer because we received a message
    timeoutTimer_.stop();
    timeoutTimer_.start();
  }
};

//------------------  SENSOR SUBSTATE BEHAVIOR ---------------------------------------------
template <typename MessageType>
class SensorTopic : public smacc::SmaccStateBehavior
{
public:
  typedef MessageType TMessageType;
  SensorComponent<MessageType> *sensor_;

  SensorTopic(std::string topicName, int queueSize = 1, ros::Duration timeout = ros::Duration(5))
  {
    this->requiresComponent(sensor_);

    // it is a little weird but the first substate behavior that passes the parameters really configure the component
    sensor_->tryStart(topicName, queueSize, timeout);
  }

  void onEntry()
  {
    // propagate the three events of the sensor component
    sensor_->OnSensorInitialMessage.connect([this](const MessageType &msg) {
      auto event = new EvSensorInitialMessage<SensorTopic<MessageType>>();
      this->postEvent(event);
    });
    sensor_->OnSensorMessage.connect([this](const MessageType &msg) {
      auto event = new EvSensorMessage<SensorTopic<MessageType>>();
      this->postEvent(event);
      this->onMessageCallback(msg);
    });

    sensor_->OnSensorTimeout.connect([this]() {
      auto event = new EvSensorMessageTimeout<SensorTopic<MessageType>>();
      this->postEvent(event);
    });
  }

  virtual void onMessageCallback(const MessageType &msg)
  {
  }

  bool onExit()
  {
  }
};
} // namespace smacc
