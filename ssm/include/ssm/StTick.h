/** @file StTick.h
 *  @brief One of two simple states for the Simple State Machine example. Uses a timer to create the transitioning event to StTock.
 *  @date 6 Mar 2020
 *  @copyright Copyright 2020 (c) GreyOrange Inc. All rights reserved.
 */
#pragma once
#ifndef __SIMPLE_STATE_MACHINE__ST_TICK_H__
#define __SIMPLE_STATE_MACHINE__ST_TICK_H__

#include <smacc/smacc.h>
// #include <ssm/SimpleStateMachine.h>
#include <ssm/EvTick.h>

namespace simple_state_machine {

class StTock;

/**
 * @brief A simple state for the Simple State Machine example.
 */
class  StTick: public smacc::SmaccState<StTick, SimpleStateMachine>
{
public:
    using SmaccState::SmaccState;

    // Transitions
    typedef boost::mpl::list<
        smacc::Transition<EvTick, StTock>  // EvGoalComplete -> StAwaitingGoal
    > reactions;

    void runtimeConfigure()
    {
        ros::NodeHandle nh;
        int time;
        getStateMachine().getGlobalSMData("time", time);

        timer_ = nh.createTimer(ros::Duration(time), &StTick::timerCallback, this);
    }

    void onEntry()
    {
        std::string message = "foo";
        getStateMachine().getGlobalSMData("message", message);
        ROS_INFO_STREAM("***** " << "TICK: " << message << " *****");
    }

private:
    ros::Timer timer_;

    void timerCallback(const ros::TimerEvent &timer_event)
    {
        timer_.stop();
        postEvent(new EvTick());
    }
};

}  // namespace simple_state_machine

#include <ssm/StTock.h>

#endif  //__SIMPLE_STATE_MACHINE__ST_TICK_H__
