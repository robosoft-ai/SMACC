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
    auto *ev = new EventType();
    stateMachine_->postEvent(ev);
}

template <typename TComponent>
TComponent *ISmaccClient::getComponent()
{
    for (auto *component : components_)
    {
        auto *tcomponent = dynamic_cast<TComponent *>(component);
        if (tcomponent != nullptr)
        {
            return tcomponent;
        }
    }

    return nullptr;
}

template <typename TComponent>
TComponent *ISmaccClient::createComponent()
{
    TComponent *component;
    stateMachine_->requiresComponent(component);
    components_.push_back(component);
    return component;
}
} // namespace smacc