#pragma once

#include <smacc/component.h>
#include <smacc/smacc_state_machine.h>

namespace smacc
{
class SmaccSubStateBehavior;
class ISmaccState;

class SmaccSubStateBehavior
{
public:
    // hapens when
    // return true to destroy the object and false to keep it alive

    // a reference to the owner state machine
    ISmaccStateMachine *stateMachine;

    // a reference to the state where the substate behavior is being executed
    ISmaccState *currentState;

    smacc::Orthogonal *currentOrthogonal;

    SmaccSubStateBehavior();

    virtual ~SmaccSubStateBehavior();

    std::string getName() const;

    virtual void onEntry();

    virtual void onExit();

    template <typename EventType>
    void postEvent(const EventType &ev);

    template <typename SmaccClientType>
    void requiresClient(SmaccClientType *&storage, bool verbose = false);

    template <typename SmaccComponentType>
    void requiresComponent(SmaccComponentType *&storage, bool verbose = false);

private:
    template <typename TDerived, typename TObjectTag>
    void assignToOrthogonal();

    friend class ISmaccState;
};
} // namespace smacc

#include <smacc/impl/smacc_substate_behavior_impl.h>