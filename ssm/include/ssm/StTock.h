/** @file StTick.h
 *  @brief One of two simple states for the Simple State Machine example. Uses a timer to create the transitioning event to StTock.
 *  @date 6 Mar 2020
 *  @copyright Copyright 2020 (c) GreyOrange Inc. All rights reserved.
 */
#pragma once
#ifndef __SIMPLE_STATE_MACHINE__ST_TOCK_H__
#define __SIMPLE_STATE_MACHINE__ST_TOCK_H__

#include <smacc/smacc.h>
// #include <ssm/SimpleStateMachine.h>
#include <ssm/EvTick.h>

namespace simple_state_machine {

class StTuck;

/**
 * @brief A simple state for the Simple State Machine example.
 */
class  StTock: public smacc::SmaccState<StTock, SimpleStateMachine>
{
public:
    using SmaccState::SmaccState;

    // Transitions
    typedef boost::mpl::list<
        smacc::Transition<EvTick, StTuck>  // EvGoalComplete -> StAwaitingGoal
    > reactions;

    void runtimeConfigure()
    {
        ros::NodeHandle nh;
        int time;
        getStateMachine().getGlobalSMData("time", time);

        timer_ = nh.createTimer(ros::Duration(time), &StTock::timerCallback, this);
    }

    void onEntry()
    {
        std::string message = "foo";
        getStateMachine().getGlobalSMData("message", message);
        ROS_INFO_STREAM("***** " << "TOCK: " << message << " *****");
    }
private:
    ros::Timer timer_;

    void timerCallback(const ros::TimerEvent &timer_event)
    {
        ROS_INFO("timerCallback");
        timer_.stop();
        postEvent(new EvTick());
    }
};

}  // namespace simple_state_machine

#include <ssm/StTuck.h>

#endif  //__SIMPLE_STATE_MACHINE__ST_TOCK_H__
