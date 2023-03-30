/*****************************************************************************************************************
 * ReelRobotix Inc. - Software License Agreement      Copyright (c) 2018
 * 	 Authors: Pablo Inigo Blasco, Brett Aldrich
 *
 ******************************************************************************************************************/
#include <smacc/smacc_state_machine.h>
#include <smacc/smacc_signal_detector.h>
#include <smacc/smacc_orthogonal.h>
#include <smacc/client_bases/smacc_action_client.h>
#include <smacc_msgs/SmaccStatus.h>
#include <smacc_msgs/SmaccTransitionLogEntry.h>
namespace smacc
{
using namespace smacc::introspection;
ISmaccStateMachine::ISmaccStateMachine(SignalDetector *signalDetector)
    : private_nh_("~"), currentState_(nullptr), stateSeqCounter_(0)
{
    ROS_INFO("Creating State Machine Base");
    signalDetector_ = signalDetector;
    signalDetector_->initialize(this);

    std::string runMode;
    if (nh_.getParam("run_mode", runMode))
    {
        if (runMode == "debug")
        {
            runMode_ = SMRunMode::DEBUG;
        }
        else if (runMode == "release")
        {
            runMode_ = SMRunMode::RELEASE;
        }
        else
        {
            ROS_ERROR("Incorrect run_mode value: %s", runMode.c_str());
        }
    }
    else
    {
        runMode_ = SMRunMode::DEBUG;
    }
}

void ISmaccStateMachine::disconnectSmaccSignalObject(void *object_ptr)
{
    ROS_INFO("[SmaccSignals] object signal disconnecting %ld", (long)object_ptr);
    if(stateCallbackConnections.count(object_ptr) > 0)
    {
        auto callbackSemaphore = stateCallbackConnections[object_ptr];
        callbackSemaphore->finalize();
        stateCallbackConnections.erase(object_ptr);
    }
    else
    {
        ROS_INFO("[SmaccSignals] no signals found %ld", (long)object_ptr);
    }
}

ISmaccStateMachine::~ISmaccStateMachine()
{
    ROS_INFO("Finishing State Machine");
}

void ISmaccStateMachine::reset()
{
}

void ISmaccStateMachine::stop()
{
}

void ISmaccStateMachine::eStop()
{
}

const std::map<std::string, std::shared_ptr<smacc::ISmaccOrthogonal>> &ISmaccStateMachine::getOrthogonals() const
{
    return this->orthogonals_;
}

void ISmaccStateMachine::updateStatusMessage()
{
    std::lock_guard<std::recursive_mutex> lock(m_mutex_);

    if (currentStateInfo_ != nullptr)
    {
        ROS_WARN_STREAM("[StateMachine] setting state active "
                        << ": " << currentStateInfo_->getFullPath());

        if (this->runMode_ == SMRunMode::DEBUG)
        {
            status_msg_.current_states.clear();
            std::list<const SmaccStateInfo *> ancestorList;
            currentStateInfo_->getAncestors(ancestorList);

            for (auto &ancestor : ancestorList)
            {
                status_msg_.current_states.push_back(ancestor->toShortName());
            }

            status_msg_.global_variable_names.clear();
            status_msg_.global_variable_values.clear();

            for (auto entry : this->globalData_)
            {
                status_msg_.global_variable_names.push_back(entry.first);
                status_msg_.global_variable_values.push_back(entry.second.first()); // <- invoke to_string()
            }

            status_msg_.header.stamp = ros::Time::now();
            status_msg_.header.frame_id = "odom";
            this->stateMachineStatusPub_.publish(status_msg_);
        }
    }
}

void ISmaccStateMachine::publishTransition(const SmaccTransitionInfo &transitionInfo)
{
    smacc_msgs::SmaccTransitionLogEntry transitionLogEntry;
    transitionLogEntry.timestamp = ros::Time::now();
    transitionInfoToMsg(transitionInfo, transitionLogEntry.transition);
    this->transitionLogHistory_.push_back(transitionLogEntry);

    transitionLogPub_.publish(transitionLogEntry);
}

void ISmaccStateMachine::onInitialize()
{
}

void ISmaccStateMachine::onInitialized()
{
    timer_ = nh_.createTimer(ros::Duration(0.5), &ISmaccStateMachine::state_machine_visualization, this);
}

void ISmaccStateMachine::initializeROS(std::string shortname)
{
    ROS_WARN_STREAM("State machine base creation:" << shortname);
    // STATE MACHINE TOPICS
    stateMachinePub_ = nh_.advertise<smacc_msgs::SmaccStateMachine>(shortname + "/smacc/state_machine_description", 1);
    stateMachineStatusPub_ = nh_.advertise<smacc_msgs::SmaccStatus>(shortname + "/smacc/status", 1);
    transitionLogPub_ = nh_.advertise<smacc_msgs::SmaccTransitionLogEntry>(shortname + "/smacc/transition_log", 1);

    // STATE MACHINE SERVICES
    transitionHistoryService_ = nh_.advertiseService(shortname + "/smacc/transition_log_history", &ISmaccStateMachine::getTransitionLogHistory, this);
}

bool ISmaccStateMachine::getTransitionLogHistory(smacc_msgs::SmaccGetTransitionHistory::Request &req, smacc_msgs::SmaccGetTransitionHistory::Response &res)
{
    ROS_WARN("Requesting Transition Log History, current size: %ld", this->transitionLogHistory_.size());
    res.history = this->transitionLogHistory_;
    return true;
}

void ISmaccStateMachine::state_machine_visualization(const ros::TimerEvent &)
{
    std::lock_guard<std::recursive_mutex> lock(m_mutex_);

    smacc_msgs::SmaccStateMachine state_machine_msg;
    state_machine_msg.states = stateMachineInfo_->stateMsgs;
    this->stateMachinePub_.publish(state_machine_msg);

    status_msg_.header.stamp = ros::Time::now();
    this->stateMachineStatusPub_.publish(status_msg_);
}

void ISmaccStateMachine::lockStateMachine(std::string msg)
{
    ROS_DEBUG("locking state machine: %s", msg.c_str());
    m_mutex_.lock();
}

void ISmaccStateMachine::unlockStateMachine(std::string msg)
{
    ROS_DEBUG("unlocking state machine: %s", msg.c_str());
    m_mutex_.unlock();
}

std::string ISmaccStateMachine::getStateMachineName()
{
    return demangleSymbol(typeid(*this).name());
}

void ISmaccStateMachine::checkStateMachineConsistence()
{
    // transition from an orthogonal that doesn’t exist.
    // transition from a source that doesn’t exist.

    // std::stringstream errorbuffer;
    // bool errorFound = false;

    // for (auto &stentry : this->stateMachineInfo_->states)
    // {
    //     auto stinfo = stentry.second;

    //     for (auto &transition : stinfo->transitions_)
    //     {
    //         auto evinfo = transition.eventInfo;
    //         bool found = false;
    //         for (auto &orthogonal : orthogonals_)
    //         {
    //             if (orthogonal.first == evinfo->getOrthogonalName())
    //             {
    //                 found = true;
    //                 break;
    //             }
    //         }

    //         if (!found)
    //         {
    //             errorbuffer << "---------" << std::endl
    //                         << "[Consistency Checking] Transition event refers not existing orthogonal." << std::endl
    //                         << "State: " << demangleType(*stinfo->tid_) << std::endl
    //                         << "Transition: " << transition.transitionTypeInfo->getFullName() << std::endl
    //                         << "Orthogonal: " << evinfo->getOrthogonalName() << std::endl
    //                         << "---------" << std::endl;

    //             errorFound = true;
    //         }
    //         //std::string getEventSourceName();
    //         //std::string getOrthogonalName();
    //     }
    // }

    // if (errorFound)
    // {
    //     ROS_WARN_STREAM("== STATE MACHINE CONSISTENCY CHECK: ==" << std::endl
    //                                                              << errorbuffer.str() << std::endl
    //                                                              << "=================");
    // }
    // cb from a client that doesn’t exist – don’t worry about making clients dynamically.
}

} // namespace smacc
