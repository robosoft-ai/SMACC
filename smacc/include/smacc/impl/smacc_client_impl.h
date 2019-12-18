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

template<typename EventType>
void ISmaccClient::postEvent()
{
    auto* ev = new EventType();
    stateMachine_->postEvent(ev);
}
} // namespace smacc