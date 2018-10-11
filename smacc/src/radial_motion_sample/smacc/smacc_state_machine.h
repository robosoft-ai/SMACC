#pragma once

#include "smacc/common.h"
#include "smacc/signal_detector.h"
#include "plugins/smacc_action_client_base.h"

#include <boost/core/demangle.hpp>
#include <map>

namespace smacc
{
struct ISmaccStateMachine
{
    public:
    ISmaccStateMachine( SignalDetector* signalDetector)
    {
        signalDetector_ = signalDetector;
        ROS_INFO("Creating State Machine Base");
        this->signalDetector_->initialize(this);
    } 

    virtual ~ISmaccStateMachine( )
    {
        ROS_INFO("Finishing State Machine");
    }

    template <typename ActionLibPlugin>
    ActionLibPlugin* requiresActionClient(std::string action_namespace)
    {
        std::string pluginkey = boost::core::demangle(typeid(ActionLibPlugin).name());
        ActionLibPlugin* ret;

        auto it = plugins_.find(pluginkey);

        ROS_INFO("requires Action Client");

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

    /// used by the actionclients when a new send goal is launched
    void registerActionClientRequest(ISmaccActionClient* client)
    {
        ROS_INFO("Registering action client request: %s", boost::core::demangle(typeid(client).name()));  
        this->signalDetector_->registerActionClientRequest(client); 
    }

    private:
        std::map<std::string, smacc::ISmaccActionClient*> plugins_;

        //event to notify to the signaldetection thread that a request has been created
        SignalDetector* signalDetector_;
};
}