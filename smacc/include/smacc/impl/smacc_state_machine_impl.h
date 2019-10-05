#pragma once

#include <smacc/smacc_state_machine.h>
#include <smacc/orthogonal.h>
#include <smacc/signal_detector.h>
#include <smacc/smacc_state_info.h>
#include <smacc/smacc_state_machine_info.h>
#include <smacc_msgs/SmaccStatus.h>
#include <sstream>

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
    this->signalDetector_->postEvent(ev);
}

template <typename StateType>
void ISmaccStateMachine::updateCurrentState(bool active, StateType *test)
{
    auto stateInfo = info_->getState<StateType>();

    if (stateInfo != nullptr)
    {
        ROS_WARN_STREAM("[StateMachine] setting state active " << active << ": " << stateInfo->getFullPath());
        stateInfo->active_ = active;

        if (this->runMode_ == SMRunMode::DEBUG)
        {
            std::list<std::shared_ptr<smacc::SmaccStateInfo>> ancestorList;
            stateInfo->getAncestors(ancestorList);

            smacc_msgs::SmaccStatus status_msg;
            for (auto &ancestor : ancestorList)
            {
                status_msg.current_states.push_back(ancestor->toShortName());
            }

            status_msg.header.stamp = ros::Time::now();
            this->statusPub_.publish(status_msg);
        }
    }
    else
    {
        ROS_ERROR_STREAM("[StateMachine] updated state not found: " << demangleSymbol(typeid(StateType).name()).c_str());
    }

    /*  
        if(a !=nullptr && a->parentState_==nullptr)
        {
            currentState_ =a;
            ROS_ERROR("really updated");
        }*/
}

} // namespace smacc