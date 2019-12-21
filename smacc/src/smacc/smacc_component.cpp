/*****************************************************************************************************************
 * ReelRobotix Inc. - Software License Agreement      Copyright (c) 2018
 * 	 Authors: Pablo Inigo Blasco, Brett Aldrich
 *
 ******************************************************************************************************************/
#include <smacc/component.h>
#include <smacc/impl/smacc_component_impl.h>
namespace smacc
{

ISmaccComponent::~ISmaccComponent()
{
}

ISmaccComponent::ISmaccComponent()
    : owner_(nullptr)
{
}

void ISmaccComponent::initialize(ISmaccComponent *owner)
{
    owner_ = owner;
}

void ISmaccComponent::setStateMachine(ISmaccStateMachine *stateMachine)
{
    stateMachine_ = stateMachine;
}

std::string ISmaccComponent::getName() const
{
    std::string keyname = demangleSymbol(typeid(*this).name());
    return keyname;
}
} // namespace smacc
