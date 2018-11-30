#include "smacc/smacc_action_client.h"
#include <boost/core/demangle.hpp>

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