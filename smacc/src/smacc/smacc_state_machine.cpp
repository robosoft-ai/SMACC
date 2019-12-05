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
ISmaccStateMachine::ISmaccStateMachine(SignalDetector *signalDetector)
    : private_nh_("~"), currentState_(nullptr)
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

ISmaccStateMachine::~ISmaccStateMachine()
{
    ROS_INFO("Finishing State Machine");
}

void ISmaccStateMachine::notifyOnStateEntry(ISmaccState *state)
{
    ROS_INFO("Notification State Entry, orthogonals: %ld", this->orthogonals_.size());
    int i = 0;
    for (auto pair : this->orthogonals_)
    {
        ROS_INFO("ortho onentry: %s", pair.second->getName().c_str());
        auto &orthogonal = pair.second;
        orthogonal->onEntry();
    }
}

void ISmaccStateMachine::notifyOnStateExit(ISmaccState *state)
{
    ROS_INFO("Notification State Exit");
    for (auto pair : this->orthogonals_)
    {
        auto &orthogonal = pair.second;
        orthogonal->onExit();
    }
}

void ISmaccStateMachine::updateStatusMessage()
{
    std::lock_guard<std::mutex> lock(m_mutex_);

    if (currentStateInfo_ != nullptr)
    {
        ROS_WARN_STREAM("[StateMachine] setting state active "
                        << ": " << currentStateInfo_->getFullPath());

        if (this->runMode_ == SMRunMode::DEBUG)
        {
            status_msg_.current_states.clear();
            std::list<smacc::SmaccStateInfo::Ptr> ancestorList;
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
            this->stateMachineStatusPub_.publish(status_msg_);
        }
    }
}

void ISmaccStateMachine::publishTransition(SmaccTransitionInfo &transitionInfo)
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

void ISmaccStateMachine::onInitializing(std::string shortname)
{
    ROS_WARN_STREAM("State machine base creation:" << shortname);
    // STATE MACHINE TOPICS
    stateMachinePub_ = nh_.advertise<smacc_msgs::SmaccStateMachine>(shortname + "/smacc/state_machine_description", 1);
    stateMachineStatusPub_ = nh_.advertise<smacc_msgs::SmaccStatus>(shortname + "/smacc/status", 1);
    transitionLogPub_ = nh_.advertise<smacc_msgs::SmaccTransitionLogEntry>(shortname + "/smacc/transition_log", 1);

    // STATE MACHINE SERVICES
    transitionHistoryService_ = nh_.advertiseService(shortname + "/smacc/transition_log_history", &ISmaccStateMachine::getTransitionLogHistory, this);

    this->onInitialize();
}

bool ISmaccStateMachine::getTransitionLogHistory(smacc_msgs::SmaccGetTransitionHistory::Request& req, smacc_msgs::SmaccGetTransitionHistory::Response& res)
{
    ROS_WARN("Requesting Transition Log History, current size: %ld", this->transitionLogHistory_.size());
    res.history = this->transitionLogHistory_;
    return true;
}

void ISmaccStateMachine::state_machine_visualization(const ros::TimerEvent &)
{

    std::lock_guard<std::mutex> lock(m_mutex_);

    smacc_msgs::SmaccStateMachine state_machine_msg;
    state_machine_msg.states = info_->stateMsgs;
    this->stateMachinePub_.publish(state_machine_msg);

    status_msg_.header.stamp = ros::Time::now();
    this->stateMachineStatusPub_.publish(status_msg_);
}

} // namespace smacc
