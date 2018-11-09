#pragma once
#include <smacc/common.h>
#include <smacc/smacc_state.h>
//-------------------------------------------------------------------------------------------------

namespace smacc
{

/// State Machine
template <typename DerivedStateMachine, typename InitialStateType>
struct SmaccStateMachineBase : public ISmaccStateMachine,  public sc::asynchronous_state_machine<DerivedStateMachine, InitialStateType, SmaccScheduler, SmaccAllocator >
{
public:
    // the node handle for this state
    ros::NodeHandle nh;
    
    SmaccStateMachineBase( my_context ctx, SignalDetector* signalDetector)
        :ISmaccStateMachine(signalDetector),
        sc::asynchronous_state_machine<DerivedStateMachine, InitialStateType, SmaccScheduler, SmaccAllocator >(ctx)
    {
    }
    
    virtual ~SmaccStateMachineBase( )
    {
    }

    // This function is defined in the Player.cpp
    virtual void initiate_impl() override
    {
        ROS_INFO("initiate_impl");
        sc::state_machine< DerivedStateMachine, InitialStateType, SmaccAllocator >::initiate();
    }
};
}