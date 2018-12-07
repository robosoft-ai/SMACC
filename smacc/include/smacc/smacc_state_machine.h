/*****************************************************************************************************************
 * ReelRobotix Inc. - Software License Agreement      Copyright (c) 2018
 * 	 Authors: Pablo Inigo Blasco, Brett Aldrich
 *
 ******************************************************************************************************************/
#pragma once

#include <smacc/common.h>
#include <smacc/smacc_action_client.h>

#include <boost/core/demangle.hpp>
#include <boost/any.hpp>
#include <map>
#include <mutex>

namespace smacc
{
// this class describes the concept of Smacc State Machine in an abastract way.
// The SmaccStateMachineBase inherits from this state machine and from 
// statechart::StateMachine<> (via multiple inheritance)
class ISmaccStateMachine
{
public:
    ISmaccStateMachine( SignalDetector* signalDetector);

    virtual ~ISmaccStateMachine();

    template <typename SmaccComponentType>
    void requiresComponent(SmaccComponentType*& storage, ros::NodeHandle nh=ros::NodeHandle())
    {
        std::lock_guard<std::mutex> lock(m_mutex_);
        std::string pluginkey = boost::core::demangle(typeid(SmaccComponentType).name());
        SmaccComponentType* ret;

        auto it = plugins_.find(pluginkey);

        if( it == plugins_.end())
        {
            ROS_INFO("%s smacc component is required. Creating a new instance.", pluginkey.c_str());

            ret = new SmaccComponentType();
            ret->init(nh);
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

    template <typename T>
    bool getGlobalSMData(std::string name, T& ret)
    {
        std::lock_guard<std::mutex> lock(m_mutex_);
        
        if(!globalData_.count(name))
        {
            return false;
        }
        else
        {
            try
            {
                boost::any v = globalData_[name];
                ret = boost::any_cast<T>(v);
                return true;    
            }
            catch(boost::bad_any_cast& ex)
            {
                return false;
            }
        }
    }

    template <typename T>
    void setGlobalSMData(std::string name, T value)
    {
        std::lock_guard<std::mutex> lock(m_mutex_);
        globalData_[name] = value;
    }

    /// used by the ISMaccActionClients when a new send goal is launched
    void registerActionClientRequest(ISmaccActionClient* component);

private:

    std::mutex m_mutex_;
    
    std::map<std::string, smacc::ISmaccComponent*> plugins_;

    std::map<std::string, boost::any> globalData_;

    //event to notify to the signaldetection thread that a request has been created
    SignalDetector* signalDetector_;
};
}