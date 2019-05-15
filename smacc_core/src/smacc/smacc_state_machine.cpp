/*****************************************************************************************************************
 * ReelRobotix Inc. - Software License Agreement      Copyright (c) 2018
 * 	 Authors: Pablo Inigo Blasco, Brett Aldrich
 *
 ******************************************************************************************************************/
#include <smacc/smacc_state_machine.h>
#include <smacc/signal_detector.h>


namespace smacc
{
ISmaccStateMachine::ISmaccStateMachine( SignalDetector* signalDetector)
{
    ROS_INFO("Creating State Machine Base");
    signalDetector_ = signalDetector;
    signalDetector_->initialize(this);
} 

ISmaccStateMachine::~ISmaccStateMachine( )
{
    ROS_INFO("Finishing State Machine");
}

/// used by the actionclients when a new send goal is launched
void ISmaccStateMachine::registerActionClientRequest(ISmaccActionClient* client)
{
    std::lock_guard<std::mutex> lock(m_mutex_);
    
    ROS_INFO("Registering action client request: %s", client->getName().c_str());  
    signalDetector_->registerActionClientRequest(client); 
}
}