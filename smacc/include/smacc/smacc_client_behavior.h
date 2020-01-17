/*****************************************************************************************************************
 * ReelRobotix Inc. - Software License Agreement      Copyright (c) 2018
 * 	 Authors: Pablo Inigo Blasco, Brett Aldrich
 *
 ******************************************************************************************************************/

#pragma once
#include <smacc/common.h>

namespace smacc
{
class ISmaccState;

class SmaccClientBehavior
{
public:
    // a reference to the owner state machine
    ISmaccStateMachine *stateMachine_;

    // a reference to the state where the client behavior is being executed
    ISmaccState *currentState;

    smacc::IOrthogonal *currentOrthogonal;

    SmaccClientBehavior();

    virtual ~SmaccClientBehavior();

    std::string getName() const;

    virtual void onEntry();

    virtual void onExit();

    template <typename EventType>
    void postEvent(const EventType &ev);

    template <typename EventType>
    void postEvent();

    template <typename SmaccClientType>
    void requiresClient(SmaccClientType *&storage);

    template <typename SmaccComponentType>
    void requiresComponent(SmaccComponentType *&storage);

private:
    template <typename TObjectTag, typename TDerived>
    void configureEventSourceTypes();

    friend class ISmaccState;
};
} // namespace smacc

#include <smacc/impl/smacc_client_behavior_impl.h>