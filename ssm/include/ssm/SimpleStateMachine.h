/** @file SimpleStateMachineImpl.h
 *  @brief The implementation of the simple state machine example
 *  @date 6 Mar 2020
 *  @copyright Copyright 2020 (c) GreyOrange Inc. All rights reserved.
 */
#pragma once
#ifndef __SIMPLE_STATE_MACHINE__SIMPLE_STATE_MACHINE_H__
#define __SIMPLE_STATE_MACHINE__SIMPLE_STATE_MACHINE_H__

#include <smacc/smacc.h>

namespace simple_state_machine {

class StTick;

class SimpleStateMachine : public smacc::SmaccStateMachineBase<SimpleStateMachine, StTick>
{
public:
    SimpleStateMachine(my_context ctx, smacc::SignalDetector *signalDetector) : smacc::SmaccStateMachineBase<SimpleStateMachine, StTick>(ctx, signalDetector)
    {}


    virtual void onInitialize()
    {
        int time = 5;
        setGlobalSMData("time", time);
        std::string msg = "Real State Machine";
        setGlobalSMData("message", msg);
    }

    virtual void unconsumed_event(const sc::event_base &evt)
    {
        ROS_WARN_STREAM("**** UNCONSUMED EVENT ****  " << smacc::demangleSymbol(typeid(evt).name()));
    }

private:
};

}  // namespace simple_state_machine

#include <ssm/StTick.h>

#endif  //__SIMPLE_STATE_MACHINE__SIMPLE_STATE_MACHINE_H__
