/*****************************************************************************************************************
 * ReelRobotix Inc. - Software License Agreement      Copyright (c) 2018
 * 	 Authors: Pablo Inigo Blasco, Brett Aldrich
 *
 ******************************************************************************************************************/
#pragma once

#include <smacc/smacc_client_behavior.h>
#include <smacc/smacc_state_machine.h>

namespace smacc
{

template <typename EventType>
void ISmaccClientBehavior::postEvent(const EventType &ev)
{
    if (stateMachine_ == nullptr)
    {
        ROS_ERROR("The client behavior cannot post events before being assigned to an orthogonal. Ignoring post event call.");
    }
    else
    {
        stateMachine_->postEvent(ev, EventLifeTime::CURRENT_STATE);
    }
}

template <typename EventType>
void ISmaccClientBehavior::postEvent()
{
    if (stateMachine_ == nullptr)
    {
        ROS_ERROR("The client behavior cannot post events before being assigned to an orthogonal. Ignoring post event call.");
    }
    else
    {

        stateMachine_->template postEvent<EventType>(EventLifeTime::CURRENT_STATE);
    }
}

//inline
ISmaccStateMachine *ISmaccClientBehavior::getStateMachine()
{
    return this->stateMachine_;
}

//inline
ISmaccState *ISmaccClientBehavior::getCurrentState()
{
    return this->currentState;
}

template <typename SmaccClientType>
void ISmaccClientBehavior::requiresClient(SmaccClientType *&storage)
{
    currentOrthogonal->requiresClient(storage);
}

template <typename SmaccClientType>
std::shared_ptr<SmaccClientType> ISmaccClientBehavior::requiresClient()
{
    return currentOrthogonal->requiresClient<SmaccClientType>();
}

template <typename SmaccComponentType>
void ISmaccClientBehavior::requiresComponent(SmaccComponentType *&storage)
{
    if (stateMachine_ == nullptr)
    {
        ROS_ERROR("Cannot use the requiresComponent funcionality before assigning the client behavior to an orthogonal. Try using the OnEntry method to capture required components.");
    }
    else
    {
        stateMachine_->requiresComponent(storage);
    }
}

template <typename SmaccComponentType>
std::shared_ptr<SmaccComponentType> ISmaccClientBehavior::requiresComponent()
{
    if (stateMachine_ == nullptr)
    {
        ROS_ERROR("Cannot use the requiresComponent funcionality before assigning the client behavior to an orthogonal. Try using the OnEntry method to capture required components.");
    }
    else
    {
        return stateMachine_->requiresComponent<SmaccComponentType>();
    }
}

template <typename TOrthogonal, typename TSourceObject>
void ISmaccClientBehavior::onOrthogonalAllocation() {}

} // namespace smacc
