/*****************************************************************************************************************
 * ReelRobotix Inc. - Software License Agreement      Copyright (c) 2018
 * 	 Authors: Pablo Inigo Blasco, Brett Aldrich
 *
 ******************************************************************************************************************/
#pragma once

#include <smacc/common.h>

#include <boost/any.hpp>
#include <map>
#include <mutex>

#include <type_traits>
#include <boost/mpl/for_each.hpp>
#include <boost/mpl/list.hpp>
#include <boost/utility/enable_if.hpp>


namespace smacc
{

class SmaccStateMachineInfo;
class SmaccStateInfo;
class Orthogonal;
class ISmaccState;


// This class describes the concept of Smacc State Machine in an abastract way.
// The SmaccStateMachineBase inherits from this state machine and from 
// statechart::StateMachine<> (via multiple inheritance)
class ISmaccStateMachine
{
    public:
    
    std::shared_ptr<SmaccStateMachineInfo> info_;
    ISmaccStateMachine( SignalDetector* signalDetector);

    virtual ~ISmaccStateMachine();

    void notifyOnStateEntry(ISmaccState* state);

    void notifyOnStateExit(ISmaccState* state);

    template <typename TOrthogonal>
    void getOrthogonal(TOrthogonal*& storage);

    template <typename SmaccComponentType>
    void requiresComponent(SmaccComponentType*& storage, ros::NodeHandle nh, std::string value);

    template <typename SmaccComponentType>
    void requiresComponent(SmaccComponentType*& storage, ros::NodeHandle nh=ros::NodeHandle())
    {
       requiresComponent(storage, nh, "");
    }

    template <typename EventType>
    void postEvent( EventType* ev);

    template <typename T>
    bool getGlobalSMData(std::string name, T& ret)
    {
        std::lock_guard<std::mutex> lock(m_mutex_);
        //ROS_WARN("get SM Data lock acquire");
        bool success = false;

        if(!globalData_.count(name))
        {
            //ROS_WARN("get SM Data - data do not exist");
            success = false;
        }
        else
        {
            //ROS_WARN("get SM DAta -data exist. accessing");
            try
            {
                boost::any v = globalData_[name];
                //ROS_WARN("get SM DAta -data exist. any cast");
                ret = boost::any_cast<T>(v);
                success = true;    
                //ROS_WARN("get SM DAta -data exist. success");
            }
            catch(boost::bad_any_cast& ex)
            {
                ROS_ERROR("bad any cast: %s", ex.what());
                success = false;
            }
        }

        //ROS_WARN("get SM Data lock release");
        return success;
    }

    template <typename T>
    void setGlobalSMData(std::string name, T value)
    {
        std::lock_guard<std::mutex> lock(m_mutex_);
        //ROS_WARN("set SM Data lock acquire");
        globalData_[name] = value;
    }

    /// Used by the ISMaccActionClients when a new send goal is launched
    void registerActionClientRequest(ISmaccActionClient* component);

    template <typename StateField, typename BehaviorType> 
    void mapBehavior()
    {
        std::string stateFieldName =demangleSymbol(typeid(StateField).name());
        std::string behaviorType =demangleSymbol(typeid(BehaviorType).name());
        ROS_INFO("Mapping state field '%s' to stateBehavior '%s'", stateFieldName.c_str(), behaviorType.c_str());
        SmaccStateBehavior* globalreference;
        if(!this->getGlobalSMData(stateFieldName,globalreference))
        {
            // Using the requires component approach, we force a unique existence
            // of this component
            BehaviorType* behavior;
            this->requiresComponent(behavior);
            globalreference =  dynamic_cast<SmaccStateBehavior*>(behavior);

            this->setGlobalSMData(stateFieldName, globalreference);
        }
    }

private:

    std::mutex m_mutex_;
    
    std::map<std::string, smacc::ISmaccComponent*> plugins_;

    std::map<std::string, boost::any> globalData_;

    std::map<std::string, smacc::Orthogonal*> orthogonals_;

    // Event to notify to the signaldetection thread that a request has been created...
    SignalDetector* signalDetector_;
};
}
