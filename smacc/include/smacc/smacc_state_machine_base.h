/*****************************************************************************************************************
 * ReelRobotix Inc. - Software License Agreement      Copyright (c) 2018
 * 	 Authors: Pablo Inigo Blasco, Brett Aldrich
 *
 ******************************************************************************************************************/
#pragma once
#include <smacc/common.h>

#include <smacc/smacc_state_base.h>
#include <smacc/smacc_state_machine.h>


namespace smacc
{

/// State Machine
template <typename DerivedStateMachine, typename InitialStateType>
struct SmaccStateMachineBase : public ISmaccStateMachine, public sc::asynchronous_state_machine<DerivedStateMachine, InitialStateType, SmaccFifoScheduler, SmaccAllocator>
{
public:
    SmaccStateMachineBase(my_context ctx, SignalDetector *signalDetector)
        : ISmaccStateMachine(signalDetector),
          sc::asynchronous_state_machine<DerivedStateMachine, InitialStateType, SmaccFifoScheduler, SmaccAllocator>(ctx)
    {
    }

    virtual ~SmaccStateMachineBase()
    {
        //updateCurrentState<InitialStateType>(false);
    }

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


    virtual void initiate_impl() override
    {
        ROS_INFO("initiate_impl");
        auto shortname = cleanShortTypeName(typeid(DerivedStateMachine));
        this->onInitializing(shortname);

        ROS_INFO("Introspecting state machine via typeWalker");
        info_ = std::make_shared<SmaccStateMachineInfo>();
        info_->buildStateMachineInfo<InitialStateType>();

        ROS_INFO("Initializing ROS communication mechanisms");
        info_->assembleSMStructureMessage(this);
        this->onInitialized();

        ROS_INFO("Initializing state machine");
        sc::state_machine<DerivedStateMachine, InitialStateType, SmaccAllocator>::initiate();
    }
};
} // namespace smacc
