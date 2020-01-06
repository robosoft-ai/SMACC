#pragma once

#include <smacc/smacc_client.h>
#include <smacc/impl/smacc_state_machine_impl.h>

namespace smacc
{
template <typename EventType>
void ISmaccClient::postEvent(const EventType &ev)
{
    stateMachine_->postEvent(ev);
}

template <typename EventType>
void ISmaccClient::postEvent()
{
    stateMachine_->postEvent<EventType>();
}

template <typename TComponent>
TComponent *ISmaccClient::getComponent()
{
    for (auto &component : components_)
    {
        auto *tcomponent = dynamic_cast<TComponent *>(component.second.get());
        if (tcomponent != nullptr)
        {
            return tcomponent;
        }
    }

    return nullptr;
}
} // namespace smacc