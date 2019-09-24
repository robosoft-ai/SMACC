#pragma once

#include <smacc/smacc_state_machine.h>
#include <smacc/orthogonal.h>
#include <smacc/signal_detector.h>
#include <smacc/smacc_state_info.h>
#include <smacc/smacc_state_machine_info.h>
#include <smacc_msgs/SmaccStatus.h>

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

//-------------------------------------------------------------------------------------------------------
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
//-------------------------------------------------------------------------------------------------------
    template <typename EventType>
    void ISmaccStateMachine::postEvent(EventType* ev)
    {
       this->signalDetector_->postEvent( ev);
    }


    template <typename StateType>
    void ISmaccStateMachine::updateCurrentState(bool active, StateType* test)
    {
        auto stateInfo =  info_->getState<StateType>();

        if(stateInfo!=nullptr)
        {
            ROS_WARN_STREAM("[StateMachine] setting state active "<< active <<": " << stateInfo->getFullPath());
            stateInfo->active_ = active;

            std::list<std::shared_ptr<smacc::SmaccStateInfo>> ancestorList;
            stateInfo->getAncestors(ancestorList);

            smacc_msgs::SmaccStatus status_msg;
            for(auto& ancestor: ancestorList)
            {
                status_msg.current_states.push_back(ancestor->toShortName());
            }

            this->statusPub_.publish(status_msg);
        }
        else
        {
            ROS_ERROR_STREAM("[StateMachine] updated state not found: " <<demangleSymbol(typeid(StateType).name()).c_str());
        }  

        
        /*  
        if(a !=nullptr && a->parentState_==nullptr)
        {
            currentState_ =a;
            ROS_ERROR("really updated");
        }*/
    }

    
}