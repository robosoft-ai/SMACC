/*****************************************************************************************************************
 * ReelRobotix Inc. - Software License Agreement      Copyright (c) 2018
 * 	 Authors: Pablo Inigo Blasco, Brett Aldrich
 *
 ******************************************************************************************************************/
#include "smacc/interface_components/smacc_action_client.h"

namespace smacc
{
using namespace actionlib;

 
ISmaccActionClient::ISmaccActionClient() 
{
}

ISmaccActionClient::~ISmaccActionClient()
{
}

void ISmaccActionClient::init(ros::NodeHandle& nh)
{
    
}

void ISmaccActionClient::init(ros::NodeHandle& nh, std::string value)
{
    name_ = nh.getNamespace();
    ROS_DEBUG("Creating Action Client %s", name_.c_str());
}

//-----------------------------------------------------------------------

ISmaccComponent::~ISmaccComponent()
{
}


ISmaccComponent::ISmaccComponent()
{
}

void ISmaccComponent::init(ros::NodeHandle& nh)
{
    
}

void ISmaccComponent::init(ros::NodeHandle& nh, std::string value)
{
    
}


void ISmaccComponent::setStateMachine(ISmaccStateMachine* stateMachine)
{
    stateMachine_ = stateMachine;
}

std::string ISmaccComponent::getName() const
{
    std::string keyname = demangleSymbol(typeid(this).name());
    return keyname;
}

}
