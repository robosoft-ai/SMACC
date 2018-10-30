#pragma once

#include "smacc/common.h"
#include "smacc/signal_detector.h"
#include "smacc/smacc_action_client.h"

#include <boost/core/demangle.hpp>
#include <boost/any.hpp>
#include <map>

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

    /// used by the actionclients when a new send goal is launched
    void registerActionClientRequest(ISmaccActionClient* client);

    template <typename ActionLibPlugin>
    ActionLibPlugin* requiresActionClient(std::string action_namespace)
    {
        std::string pluginkey = boost::core::demangle(typeid(ActionLibPlugin).name());
        ActionLibPlugin* ret;

        auto it = plugins_.find(pluginkey);

        if( it == plugins_.end())
        {
            ROS_INFO("%s resource is required. Creating a new instance.", pluginkey.c_str());

            ret = new ActionLibPlugin(action_namespace);
            ret->setStateMachine(this);
            plugins_ [pluginkey] = static_cast<smacc::ISmaccActionClient*>(ret);
            ROS_INFO("%s resource is required. Done.", pluginkey.c_str());

        }
        else
        {
            ROS_INFO("%s resource is required. Found resource in cache.", pluginkey.c_str());
            ret = dynamic_cast<ActionLibPlugin*>(it->second);
        }
    
        return ret;
    }

    template <typename T>
    bool getData(std::string name, T& ret)
    {
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
    void setData(std::string name, T value)
    {
        globalData_[name] = value;
    }

private:
    std::map<std::string, smacc::ISmaccActionClient*> plugins_;

    std::map<std::string, boost::any> globalData_;

    //event to notify to the signaldetection thread that a request has been created
    SignalDetector* signalDetector_;
};
}