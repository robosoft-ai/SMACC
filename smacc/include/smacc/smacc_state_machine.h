/*****************************************************************************************************************
 * ReelRobotix Inc. - Software License Agreement      Copyright (c) 2018
 * 	 Authors: Pablo Inigo Blasco, Brett Aldrich
 *
 ******************************************************************************************************************/
#pragma once

#include <smacc/common.h>
#include <smacc/reflection.h>
#include <smacc/smacc_state.h>
#include <smacc/logic_units/logic_unit_base.h>

#include <boost/any.hpp>
#include <map>
#include <mutex>

#include <type_traits>
#include <boost/mpl/for_each.hpp>
#include <boost/mpl/list.hpp>
#include <boost/utility/enable_if.hpp>
#include <smacc_msgs/SmaccStatus.h>
#include <smacc_msgs/SmaccStateMachine.h>

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
    ISmaccStateMachine(SignalDetector *signalDetector);

    virtual ~ISmaccStateMachine();

    virtual void Reset()
    {
    }

    virtual void Stop()
    {
    }

    virtual void EStop()
    {
    }

    void notifyOnStateEntry(ISmaccState *state);

    void notifyOnStateExit(ISmaccState *state);

    template <typename TOrthogonal>
    bool getOrthogonal(std::shared_ptr<TOrthogonal> &storage);

    const std::map<std::string, std::shared_ptr<smacc::Orthogonal>> &getOrthogonals() const
    {
        return this->orthogonals_;
    }

    template <typename SmaccComponentType>
    void requiresComponent(SmaccComponentType *&storage, bool verbose);

    template <typename EventType>
    void postEvent(EventType *ev);

    template <typename T>
    bool getGlobalSMData(std::string name, T &ret)
    {
        std::lock_guard<std::mutex> lock(m_mutex_);
        //ROS_WARN("get SM Data lock acquire");
        bool success = false;

        if (!globalData_.count(name))
        {
            //ROS_WARN("get SM Data - data do not exist");
            success = false;
        }
        else
        {
            //ROS_WARN("get SM DAta -data exist. accessing");
            try
            {
                auto &v = globalData_[name];

                //ROS_WARN("get SM DAta -data exist. any cast");
                ret = boost::any_cast<T>(v.second);
                success = true;
                //ROS_WARN("get SM DAta -data exist. success");
            }
            catch (boost::bad_any_cast &ex)
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
        {
            std::lock_guard<std::mutex> lock(m_mutex_);
            //ROS_WARN("set SM Data lock acquire");

            globalData_[name] = {
                [this, name]() {
                    std::stringstream ss;
                    auto val = any_cast<T>(globalData_[name].second);
                    ss << val;
                    return ss.str();
                },
                value};
        }

        this->updateStatusMessage();
    }

    /// Used by the ISMaccActionClients when a new send goal is launched
    void registerActionClientRequest(ISmaccActionClient *component);

    template <typename StateField, typename BehaviorType>
    void mapBehavior()
    {
        std::string stateFieldName = demangleSymbol(typeid(StateField).name());
        std::string behaviorType = demangleSymbol(typeid(BehaviorType).name());
        ROS_INFO("Mapping state field '%s' to stateBehavior '%s'", stateFieldName.c_str(), behaviorType.c_str());
        SmaccSubStateBehavior *globalreference;
        if (!this->getGlobalSMData(stateFieldName, globalreference))
        {
            // Using the requires component approach, we force a unique existence
            // of this component
            BehaviorType *behavior;
            this->requiresComponent(behavior);
            globalreference = dynamic_cast<SmaccSubStateBehavior *>(behavior);

            this->setGlobalSMData(stateFieldName, globalreference);
        }
    }

    template <typename StateType>
    void updateCurrentState(bool active, StateType *test);

    void updateStatusMessage();

    std::string getStateMachineName()
    {
        return demangleSymbol(typeid(*this).name());
    }

    void state_machine_visualization(const ros::TimerEvent &);

    inline ISmaccState *getCurrentState() { return currentState_; };

    inline std::shared_ptr<smacc::SmaccStateInfo> getCurrentStateInfo() { return currentStateInfo_; }

    void publishTransition(SmaccTransitionInfo &transitionInfo);

protected:
    void initializeRosComponents();

    template <typename TOrthogonal>
    void createOrthogonal();

    // Delegates to ROS param access with the current NodeHandle
    template <typename T>
    bool getParam(std::string param_name, T &param_storage)
    {
        return nh_.getParam(param_name, param_storage);
    }

    // Delegates to ROS param access with the current NodeHandle
    template <typename T>
    void setParam(std::string param_name, T param_val)
    {
        return nh_.setParam(param_name, param_val);
    }

    // Delegates to ROS param access with the current NodeHandle
    template <typename T>
    bool param(std::string param_name, T &param_val, const T &default_val) const
    {
        return nh_.param(param_name, param_val, default_val);
    }

    // The node handle for this state
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;

    ros::Timer timer_;
    ros::Publisher stateMachinePub_;
    ros::Publisher stateMachineStatusPub_;
    ros::Publisher transitionLogPub_;

    ISmaccState *currentState_;
    std::shared_ptr<smacc::SmaccStateInfo> currentStateInfo_;

    smacc_msgs::SmaccStatus status_msg_;

private:
    std::mutex m_mutex_;

    // components
    std::map<std::string, smacc::ISmaccComponent *> plugins_;

    // shared variables
    std::map<std::string, std::pair<std::function<std::string()>, boost::any>> globalData_;

    // orthogonals
    std::map<std::string, std::shared_ptr<smacc::Orthogonal>> orthogonals_;

    smacc::SMRunMode runMode_;

    // Event to notify to the signaldetection thread that a request has been created...
    SignalDetector *signalDetector_;

public:
    std::shared_ptr<SmaccStateMachineInfo> info_;
};
} // namespace smacc

#include <smacc/impl/smacc_state_impl.h>
#include <smacc/impl/smacc_client_impl.h>
#include <smacc/impl/smacc_component_impl.h>
#include <smacc/impl/smacc_orthogonal_impl.h>