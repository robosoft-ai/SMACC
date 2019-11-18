#pragma once

#include <smacc/smacc_state_machine.h>
#include <smacc/smacc_state.h>
#include <smacc/orthogonal.h>
#include <smacc/signal_detector.h>
#include <smacc/reflection.h>
#include <smacc/smacc_state_machine_info.h>
#include <smacc_msgs/SmaccStatus.h>
#include <sstream>
#include <smacc/logic_units/logic_unit_base.h>

namespace smacc
{
template <typename TOrthogonal>
bool ISmaccStateMachine::getOrthogonal(std::shared_ptr<TOrthogonal> &storage)
{
    std::lock_guard<std::mutex> lock(m_mutex_);
    std::string orthogonalkey = demangledTypeName<TOrthogonal>();
    std::shared_ptr<TOrthogonal> ret;

    auto it = orthogonals_.find(orthogonalkey);

    if (it != orthogonals_.end())
    {
        ROS_INFO("%s resource is required. Found resource in cache.", orthogonalkey.c_str());
        ret = dynamic_pointer_cast<TOrthogonal>(it->second);
        storage = ret;
        return true;
    }
    else
    {
        std::stringstream ss;
        ss << "Orthogonal not found " << orthogonalkey.c_str() << std::endl;
        ss << "The existing orthogonals are the following: " << std::endl;
        for (auto &orthogonal : orthogonals_)
        {
            ss << " - " << orthogonal.first << std::endl;
        }

        ROS_WARN_STREAM(ss.str());

        return false;
    }
}

//-------------------------------------------------------------------------------------------------------
template <typename TOrthogonal>
void ISmaccStateMachine::createOrthogonal()
{
    std::string orthogonalkey = demangledTypeName<TOrthogonal>();

    if (orthogonals_.count(orthogonalkey) == 0)
    {
        auto ret = std::make_shared<TOrthogonal>();
        orthogonals_[orthogonalkey] = dynamic_pointer_cast<smacc::Orthogonal>(ret);

        ret->setStateMachine(this);

        ROS_INFO("%s Orthogonal is created", orthogonalkey.c_str());
    }
    else
    {
        ROS_WARN_STREAM("There were already one existing orthogonal of type " << orthogonalkey.c_str() << ". Skipping creation orthogonal request. ");
        std::stringstream ss;
        ss << "The existing orthogonals are the following: " << std::endl;
        for (auto &orthogonal : orthogonals_)
        {
            ss << " - " << orthogonal.first << std::endl;
        }
        ROS_WARN_STREAM(ss.str());
    }
}

//-------------------------------------------------------------------------------------------------------
template <typename SmaccComponentType>
void ISmaccStateMachine::requiresComponent(SmaccComponentType *&storage, bool verbose)
{
    ROS_INFO("component %s is required", demangleSymbol(typeid(SmaccComponentType).name()).c_str());
    std::lock_guard<std::mutex> lock(m_mutex_);
    std::string pluginkey = demangledTypeName<SmaccComponentType>();
    SmaccComponentType *ret;

    auto it = plugins_.find(pluginkey);

    if (it == plugins_.end())
    {
        ROS_INFO("%s smacc component is required. Creating a new instance.", pluginkey.c_str());

        ret = new SmaccComponentType();
        ret->setStateMachine(this);
        plugins_[pluginkey] = static_cast<smacc::ISmaccComponent *>(ret);
        ROS_INFO("%s resource is required. Done.", pluginkey.c_str());
    }
    else
    {
        ROS_INFO("%s resource is required. Found resource in cache.", pluginkey.c_str());
        ret = dynamic_cast<SmaccComponentType *>(it->second);
    }

    storage = ret;
}
//-------------------------------------------------------------------------------------------------------
template <typename EventType>
void ISmaccStateMachine::postEvent(EventType *ev)
{
    // when a postting event is requested by any component, client, or substate behavior
    // we reach this place. Now, we propagate the events to all the state logic units to generate
    // some more events
    if (currentState_ != nullptr)
    {
        ROS_INFO_STREAM("EVENT: " << demangleSymbol<EventType>());
        for (auto &lu : currentState_->logicUnits_)
        {
            lu->notifyEvent(ev);
        }
    }
    else
    {
        ROS_INFO_THROTTLE(0.5, "debug post event, to lu, but currentState was nullptr");
    }

    this->signalDetector_->postEvent(ev);
}
//-------------------------------------------------------------------------------------------------------
template <typename StateType>
void ISmaccStateMachine::updateCurrentState(bool active, StateType *currentState)
{
    currentState_ = currentState;

    currentStateInfo_ = info_->getState<StateType>();
    this->updateStatusMessage();
}
} // namespace smacc