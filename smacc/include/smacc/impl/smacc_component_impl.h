/*****************************************************************************************************************
 * ReelRobotix Inc. - Software License Agreement      Copyright (c) 2018
 * 	 Authors: Pablo Inigo Blasco, Brett Aldrich
 *
 ******************************************************************************************************************/

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

    template <typename TComponent>
    void ISmaccComponent::requiresComponent(TComponent *&requiredComponentStorage)
    {
        requiredComponentStorage = this->owner_->getComponent<TComponent>();
    }

    template <typename TClient>
    void ISmaccComponent::requiresClient(TClient *& requiredClientStorage)
    {
        this->owner_->requiresClient(requiredClientStorage);
    }
} // namespace smacc