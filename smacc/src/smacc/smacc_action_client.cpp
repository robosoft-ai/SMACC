#include "smacc/smacc_action_client.h"
#include <boost/core/demangle.hpp>

namespace smacc
{
using namespace actionlib;

 
ISmaccActionClient::ISmaccActionClient(std::string action_client_namespace) 
{
    name_ = action_client_namespace;
    ROS_DEBUG("Creating Action Client %s", action_client_namespace.c_str());
}

ISmaccActionClient::~ISmaccActionClient()
{
}

//-----------------------------------------------------------------------

ISmaccComponent::~ISmaccComponent()
{
}

void ISmaccComponent::setStateMachine(ISmaccStateMachine* stateMachine)
{
    stateMachine_ = stateMachine;
}

std::string ISmaccComponent::getName() const
{
    std::string keyname = boost::core::demangle(typeid(this).name());
    return keyname;
}

}