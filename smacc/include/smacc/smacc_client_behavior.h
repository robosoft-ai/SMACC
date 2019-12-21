#pragma once
#include <smacc/common.h>

namespace smacc
{
class ISmaccState;

class SmaccClientBehavior
{
public:
    // hapens when
    // return true to destroy the object and false to keep it alive

    // a reference to the owner state machine
    ISmaccStateMachine *stateMachine;

    // a reference to the state where the client behavior is being executed
    ISmaccState *currentState;

    smacc::Orthogonal *currentOrthogonal;

    SmaccClientBehavior();

    virtual ~SmaccClientBehavior();

    std::string getName() const;

    virtual void onEntry();

    virtual void onExit();

    template <typename EventType>
    void postEvent(const EventType &ev);

    template <typename SmaccClientType>
    void requiresClient(SmaccClientType *&storage);

    template <typename SmaccComponentType>
    void requiresComponent(SmaccComponentType *&storage);

private:
    template <typename TDerived, typename TObjectTag>
    void configureEventSourceTypes();

    friend class ISmaccState;
};
} // namespace smacc

#include <smacc/impl/smacc_client_behavior_impl.h>