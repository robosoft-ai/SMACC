#pragma once

#include <smacc_core/smacc_state.h>
#include <boost/statechart/event.hpp>

#include <ros/ros.h>
#include <ros/duration.h>

namespace smacc
{
class Timer;

//----------------- TIMER EVENT DEFINITION ----------------------------------------------
struct TimerTickEvent: sc::event< TimerTickEvent>
{
  Timer* sender;
  ros::TimerEvent timedata;

  TimerTickEvent(Timer* sender, const ros::TimerEvent& timedata)
  {
    this->sender = sender;
    this->timedata = timedata;
  }
};

//------------------  TIMER SUBSTATE ---------------------------------------------

class Timer : public smacc::SmaccStateBehavior
{
  
 public:
  typedef TimerTickEvent TickEvent;

  ros::NodeHandle nh;
  ros::Timer timer;
  ros::Duration duration;
  bool oneshot;

  Timer(ros::Duration duration, bool oneshot = false)
  {
    this->duration = duration;
    this->oneshot = oneshot;
  } 

  void onEntry()
  {
    timer = nh.createTimer(duration, boost::bind(&Timer::timerCallback, this, _1),oneshot);
  }

  bool onExit()
  {
    timer.stop();
  }

  void timerCallback(const ros::TimerEvent& timedata)
  {
    TimerTickEvent* event= new TimerTickEvent(this, timedata);
    this->postEvent(event);

  }   
};
}
