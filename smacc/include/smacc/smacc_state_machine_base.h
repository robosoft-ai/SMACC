/*****************************************************************************************************************
 * ReelRobotix Inc. - Software License Agreement      Copyright (c) 2018
 * 	 Authors: Pablo Inigo Blasco, Brett Aldrich
 *
 ******************************************************************************************************************/
#pragma once
#include <smacc/common.h>

#include <smacc/smacc_state_base.h>
#include <smacc/reflection.h>
#include <smacc/smacc_state_machine_info.h>
#include <smacc/smacc_state_machine.h>

#include <smacc/logic_units/logic_unit_base.h>
#include <smacc_msgs/SmaccTransitionLogEntry.h>
//-------------------------------------------------------------------------------------------------

namespace smacc
{

/// State Machine
template <typename DerivedStateMachine, typename InitialStateType>
struct SmaccStateMachineBase : public ISmaccStateMachine, public sc::asynchronous_state_machine<DerivedStateMachine, InitialStateType, SmaccScheduler, SmaccAllocator>
{
public:
    //std::vector<std::shared_ptr<SmaccStateInfo>> currentState_;

    virtual void Reset() override
    {
        ISmaccStateMachine::Reset();
        this->terminate();
        smacc::run<DerivedStateMachine>();
    }

    virtual void Stop() override
    {
        ISmaccStateMachine::Stop();
        this->terminate();
    }

    virtual void EStop() override
    {
        ISmaccStateMachine::EStop();
        this->terminate();
    }

    SmaccStateMachineBase(my_context ctx, SignalDetector *signalDetector)
        : ISmaccStateMachine(signalDetector),
          sc::asynchronous_state_machine<DerivedStateMachine, InitialStateType, SmaccScheduler, SmaccAllocator>(ctx)
    {
        auto shortname = cleanShortTypeName(typeid(DerivedStateMachine));
        ROS_WARN_STREAM("State machine base creation:" << shortname);
        // STATE MACHINE TOPICS
        stateMachinePub_ = nh_.advertise<smacc_msgs::SmaccStateMachine>(shortname + "/smacc/state_machine_description", 1);
        stateMachineStatusPub_ = nh_.advertise<smacc_msgs::SmaccStatus>(shortname + "/smacc/status", 1);
        transitionLogPub_ = nh_.advertise<smacc_msgs::SmaccTransitionLogEntry>(shortname + "/smacc/transition_log", 1);
    }

    virtual void onInitialize()
    {
    }

    virtual ~SmaccStateMachineBase()
    {
        //updateCurrentState<InitialStateType>(false);
    }

    virtual void initiate_impl() override
    {
        ROS_INFO("initiate_impl");
        this->onInitialize();

        info_ = std::make_shared<SmaccStateMachineInfo>();
        info_->buildStateMachineInfo<InitialStateType>();

        info_->assembleSMStructureMessage(this);
        this->initializeRosComponents();

        sc::state_machine<DerivedStateMachine, InitialStateType, SmaccAllocator>::initiate();        
    }
};
} // namespace smacc
