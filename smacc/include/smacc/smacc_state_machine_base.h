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

    virtual void reset() override
    {
        ISmaccStateMachine::reset();
        this->terminate();
        smacc::run<DerivedStateMachine>();
    }

    virtual void stop() override
    {
        ISmaccStateMachine::stop();
        this->terminate();
    }

    virtual void eStop() override
    {
        ISmaccStateMachine::eStop();
        this->terminate();
    }

    virtual void initiate_impl() override
    {
        // this is before because this creates orthogonals
        this->onInitialize();

        ROS_INFO("[SmaccStateMachine] Introspecting state machine via typeWalker");
        this->buildStateMachineInfo<InitialStateType>();

        ROS_INFO("[SmaccStateMachine] initiate_impl");
        auto shortname =  smacc::utils::cleanShortTypeName(typeid(DerivedStateMachine));

        this->initializeROS(shortname);        

        ROS_INFO("[SmaccStateMachine] Initializing ROS communication mechanisms");
        this->onInitialized();

        ROS_INFO("[SmaccStateMachine] Initializing state machine");
        sc::state_machine<DerivedStateMachine, InitialStateType, SmaccAllocator>::initiate();
    }
};
} // namespace smacc
