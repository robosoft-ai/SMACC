#include "smacc/smacc_state_machine.h"


namespace smacc
{
ISmaccStateMachine::ISmaccStateMachine( SignalDetector* signalDetector)
{
    signalDetector_ = signalDetector;
    ROS_INFO("Creating State Machine Base");
    signalDetector_->initialize(this);
} 

ISmaccStateMachine::~ISmaccStateMachine( )
{
    ROS_INFO("Finishing State Machine");
}

/// used by the actionclients when a new send goal is launched
void ISmaccStateMachine::registerActionClientRequest(ISmaccActionClient* client)
{
    ROS_INFO("Registering action client request: %s", client->getName().c_str());  
    signalDetector_->registerActionClientRequest(client); 
}
}