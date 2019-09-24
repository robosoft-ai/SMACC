    #pragma once

    #include <smacc/component.h>
    #include <smacc/impl/smacc_state_machine_impl.h>

    namespace smacc
    {
    template <typename EventType>
    void ISmaccComponent::postEvent(const EventType &ev)
    {
        stateMachine_->postEvent(ev);
    }
    }