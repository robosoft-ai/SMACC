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

template <typename SmaccComponentType, typename... TArgs>
SmaccComponentType *ISmaccClient::createComponent(TArgs... targs)
{
    const std::type_info *componentkey = &typeid(SmaccComponentType);
    std::shared_ptr<SmaccComponentType> ret;

    auto it = components_.find(componentkey);

    if (it == components_.end())
    {
        auto tname = demangledTypeName<SmaccComponentType>();
        ROS_DEBUG("%s smacc component is required. Creating a new instance.", tname.c_str());

        ret = std::shared_ptr<SmaccComponentType>(new SmaccComponentType(targs...));
        ret->initialize(this);
        ret->setStateMachine(stateMachine_);

        components_[componentkey] = ret; //std::dynamic_pointer_cast<smacc::ISmaccComponent>(ret);
        ROS_DEBUG("%s resource is required. Done.", tname.c_str());
    }
    else
    {
        ROS_DEBUG("%s resource is required. Found resource in cache.", demangledTypeName<SmaccComponentType>().c_str());
        ret = dynamic_pointer_cast<SmaccComponentType>(it->second);
    }

    return ret.get();
}
} // namespace smacc