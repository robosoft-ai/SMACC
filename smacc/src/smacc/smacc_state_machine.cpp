/*****************************************************************************************************************
 * ReelRobotix Inc. - Software License Agreement      Copyright (c) 2018
 * 	 Authors: Pablo Inigo Blasco, Brett Aldrich
 *
 ******************************************************************************************************************/
#include <smacc/smacc_state_machine.h>
#include <smacc/signal_detector.h>
#include <smacc/orthogonal.h>
#include <smacc/interface_components/smacc_action_client.h>
#include <smacc_msgs/SmaccStatus.h>

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

/// used by the actionclients when a new send goal is launched
void ISmaccStateMachine::registerActionClientRequest(ISmaccActionClient *client)
{
    std::lock_guard<std::mutex> lock(m_mutex_);

    ROS_INFO("Registering action client request: %s", client->getName().c_str());
    signalDetector_->registerActionClientRequest(client);
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
} // namespace smacc