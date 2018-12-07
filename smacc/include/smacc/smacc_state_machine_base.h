/*****************************************************************************************************************
 * ReelRobotix Inc. - Software License Agreement      Copyright (c) 2018
 * 	 Authors: Pablo Inigo Blasco, Brett Aldrich
 *
 ******************************************************************************************************************/
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
        nh = ros::NodeHandle(cleanTypeName(typeid(DerivedStateMachine)));
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

     // delegates to ROS param access with the current NodeHandle
    template <typename T>
    bool getParam(std::string param_name, T& param_storage)
    {
        return nh.getParam(param_name, param_storage);
    }

    // delegates to ROS param access with the current NodeHandle
    template <typename T>
    void setParam(std::string param_name, T param_val)
    {
        return nh.setParam(param_name, param_val);
    }

    // delegates to ROS param access with the current NodeHandle
    template<typename T>
    bool param(std::string param_name, T& param_val, const T& default_val) const
    {
        return nh.param(param_name, param_val, default_val);
    }
};
}