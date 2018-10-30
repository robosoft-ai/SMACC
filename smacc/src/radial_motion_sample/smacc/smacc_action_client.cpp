#include "smacc/smacc_action_client.h"

namespace smacc
{
using namespace actionlib;
        
ISmaccActionClient::~ISmaccActionClient()
{
}

void ISmaccActionClient::setStateMachine(ISmaccStateMachine* stateMachine)
{
    stateMachine_ = stateMachine;
}

ISmaccActionClient::ISmaccActionClient(std::string action_client_namespace) 
{
    name_ = action_client_namespace;
    ROS_DEBUG("Creating Action Client %s", action_client_namespace.c_str());
}            
}