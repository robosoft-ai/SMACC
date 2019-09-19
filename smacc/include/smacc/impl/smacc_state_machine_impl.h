#pragma once

#include <smacc/smacc_state_machine.h>
#include <smacc/orthogonal.h>
#include <smacc/signal_detector.h>

namespace smacc
{
    template <typename TOrthogonal>
    void ISmaccStateMachine::getOrthogonal(TOrthogonal*& storage)
    {
        std::lock_guard<std::mutex> lock(m_mutex_);
        std::string orthogonalkey = demangledTypeName<TOrthogonal>();
        TOrthogonal* ret;

        auto it = plugins_.find(orthogonalkey);

        if( it == plugins_.end())
        {
            ret = new TOrthogonal();
            ret->setStateMachine(this);
            orthogonals_ [orthogonalkey] = static_cast<smacc::Orthogonal*>(ret);
            ROS_INFO("%s Orthogonal is created", orthogonalkey.c_str());
        }
        else
        {
            ROS_INFO("%s resource is required. Found resource in cache.", orthogonalkey.c_str());
            ret = dynamic_cast<TOrthogonal*>(it->second);
        }

        storage = ret;
    }


    template <typename SmaccComponentType>
    void ISmaccStateMachine::requiresComponent(SmaccComponentType*& storage, ros::NodeHandle nh, std::string value, bool verbose)
    {
        ROS_INFO("component %s is required", demangleSymbol(typeid(SmaccComponentType).name()).c_str());
        std::lock_guard<std::mutex> lock(m_mutex_);
        std::string pluginkey = demangledTypeName<SmaccComponentType>();
        SmaccComponentType* ret;

        auto it = plugins_.find(pluginkey);

        if( it == plugins_.end())
        {
            ROS_INFO("%s smacc component is required. Creating a new instance.", pluginkey.c_str());

            ret = new SmaccComponentType();
            ret->init(nh, value);
            ret->setStateMachine(this);
            plugins_ [pluginkey] = static_cast<smacc::ISmaccComponent*>(ret);
            ROS_INFO("%s resource is required. Done.", pluginkey.c_str());
        }
        else
        {
            ROS_INFO("%s resource is required. Found resource in cache.", pluginkey.c_str());
            ret = dynamic_cast<SmaccComponentType*>(it->second);
        }
    
        storage = ret;
    }

    template <typename EventType>
    void ISmaccStateMachine::postEvent(EventType* ev)
    {
       this->signalDetector_->postEvent( ev);
    }

    
}