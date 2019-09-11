#pragma once

#include <smacc/smacc_state.h>
#include <boost/statechart/event.hpp>

#include <ros/ros.h>
#include <ros/duration.h>

namespace smacc
{

//----------------- TIMER EVENT DEFINITION ----------------------------------------------
template<typename SensorBehaviorType>
struct EvSensorInitialMessage: sc::event<EvSensorInitialMessage<SensorBehaviorType>>
{
    //typename EvSensorInitialMessage<SensorBehaviorType>::TMessageType msgData;
};

template<typename SensorBehaviorType>
struct EvSensorMessage: sc::event<EvSensorMessage<SensorBehaviorType>>
{
    //typename EvSensorInitialMessage<SensorBehaviorType>::TMessageType msgData;
};

template<typename SensorBehaviorType>
struct EvSensorMessageTimeout: sc::event<EvSensorMessageTimeout<SensorBehaviorType>>
{
};

struct testEvent: sc::event<testEvent>
{

};


//------------------  TIMER SUBSTATE ---------------------------------------------

template <typename MessageType>
class SensorTopic : public smacc::SmaccStateBehavior
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

  SensorTopic(std::string topicName, int queueSize = 1, ros::Duration timeout= ros::Duration(5))
  {
    topicName_ = topicName;
    queueSize_ = queueSize;
    firstTime_ = true;
    timeoutDuration_ = timeout;
  } 

  void onEntry()
  {
     ROS_INFO_STREAM("Subscribing to sensor topic: " << topicName_);
     sub_ = nh_.subscribe(topicName_, queueSize_, &SensorTopic<MessageType>::messageCallback, this);
     timeoutTimer_ = nh_.createTimer(this->timeoutDuration_,boost::bind(&SensorTopic<MessageType>::timeoutCallback, this, _1));
     timeoutTimer_.start();
  }

  bool onExit()
  {
     sub_.shutdown();
     timeoutTimer_.stop();
  }

  void timeoutCallback(const ros::TimerEvent& ev)
  {
    auto event= new EvSensorMessageTimeout<SensorTopic<MessageType>>();
    this->postEvent(event);
  }

  virtual void onMessageCallback(const MessageType& msg)
  {

    // empty to fill by sensor customization based on inheritance
  }

  void messageCallback(const MessageType& msg)
  {
    //ROS_INFO_STREAM("message received from: "<< this->topicName_);

    if(firstTime_)
    {
      firstTime_ = false;
      auto event= new EvSensorInitialMessage<SensorTopic<MessageType>>();
      this->postEvent(event);
    }
    

    EvSensorMessage<SensorTopic<MessageType>>* ev2= new EvSensorMessage<SensorTopic<MessageType>>();
    this->postEvent(ev2);
    
    this->onMessageCallback(msg);

    timeoutTimer_.stop();
    timeoutTimer_.start();
  }   
};
}
