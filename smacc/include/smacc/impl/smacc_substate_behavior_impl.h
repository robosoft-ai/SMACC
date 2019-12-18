#pragma once

#include <smacc/smacc_substate_behavior.h>
#include <smacc/smacc_state_machine.h>

namespace smacc
{

template <typename EventType>
void SmaccSubStateBehavior::postEvent(const EventType &ev)
{
    if (stateMachine == nullptr)
    {
        ROS_ERROR("The substate behavior cannot post events before being assigned to an orthogonal. Ignoring post event call.");
    }
    else
    {
        stateMachine->postEvent(ev);
    }
}

template <typename SmaccClientType>
void SmaccSubStateBehavior::requiresClient(SmaccClientType *&storage, bool verbose)
{
    currentOrthogonal->requiresClient(storage, verbose);
}

template <typename SmaccComponentType>
void SmaccSubStateBehavior::requiresComponent(SmaccComponentType *&storage, bool verbose)
{
    if (stateMachine == nullptr)
    {
        ROS_ERROR("Cannot use the requiresComponent funcionality before asigning the substate behavior to an orthogonal. Try using the OnEntry method to capture required components.");
    }
    else
    {
        stateMachine->requiresComponent(storage, verbose);
    }
}

template <typename TDerived, typename TObjectTag>
void SmaccSubStateBehavior::assignToOrthogonal() {}

} // namespace smacc