    #pragma once
    #include <smacc/component.h>
    
    namespace smacc
    {
    template <typename EventType>
    void ISmaccComponent::postEvent(const EventType &ev)
    {
        stateMachine_->postEvent(ev);
    }
    }