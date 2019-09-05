#pragma once

#include <smacc/smacc_state.h>
#include <boost/statechart/event.hpp>

#include <ros/ros.h>
#include <ros/duration.h>

namespace smacc
{

//----------------- TIMER EVENT DEFINITION ----------------------------------------------
template<typename SensorBehaviorType>
struct SensorInitialMessage: sc::event<SensorInitialMessage<SensorBehaviorType>>
{
};

template<typename SensorBehaviorType>
struct SensorMessage: sc::event<SensorMessage<SensorBehaviorType>>
{
};

template<typename SensorBehaviorType>
struct SensorMessageTimeout: sc::event<SensorMessageTimeout<SensorBehaviorType>>
{
};

//------------------  TIMER SUBSTATE ---------------------------------------------

template <typename MessageType>
class SensorTopic : public smacc::SmaccStateBehavior
{
  
 public:
  typedef SensorInitialMessage<SensorTopic<MessageType>> InitialMessageEvent;
  typedef SensorMessageTimeout<SensorTopic<MessageType>> MessageTimeoutEvent;
  typedef SensorMessage<SensorTopic<MessageType>> MessageEvent;

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
    auto event= new MessageTimeoutEvent();
    this->postEvent(event);
  }

  virtual void onMessageCallback(const MessageType& msg)
  {
    // empty to fill by sensor customization based on inheritance
  }

  void messageCallback(const MessageType& msg)
  {
    if(firstTime_)
    {
      firstTime_ == false;

      auto event= new InitialMessageEvent();
      this->postEvent(event);
    }
    
    this->onMessageCallback(msg);

    timeoutTimer_.stop();
    timeoutTimer_.start();
  }   
};
}
