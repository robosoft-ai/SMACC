/*****************************************************************************************************************
 * ReelRobotix Inc. - Software License Agreement      Copyright (c) 2018
 * 	 Authors: Pablo Inigo Blasco, Brett Aldrich
 *
 ******************************************************************************************************************/

#pragma once
#include <smacc/smacc_client_behavior_base.h>

namespace smacc
{
class SmaccClientBehavior: public ISmaccClientBehavior
{
public:
    SmaccClientBehavior();

    virtual ~SmaccClientBehavior();

    inline ISmaccStateMachine *getStateMachine();

    std::string getName() const;

    virtual void runtimeConfigure();

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

    inline ISmaccState *getCurrentState();

private:
    template <typename TObjectTag, typename TDerived>
    void configureEventSourceTypes();

    // a reference to the owner state machine
    ISmaccStateMachine *stateMachine_;

    // a reference to the state where the client behavior is being executed
    ISmaccState *currentState;

    smacc::ISmaccOrthogonal *currentOrthogonal;

    friend class ISmaccState;
    friend class ISmaccOrthogonal;
};
} // namespace smacc

#include <smacc/impl/smacc_client_behavior_impl.h>