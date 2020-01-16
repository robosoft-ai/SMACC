#pragma once
#include <smacc/smacc_state.h>
#include <smacc/smacc_orthogonal.h>
#include <smacc/smacc_client_behavior.h>
#include <smacc/smacc_state_behavior.h>
#include <smacc/introspection/string_type_walker.h>
#include <smacc/smacc_client_behavior.h>
#include <smacc/smacc_state_machine.h>

namespace smacc
{
using namespace smacc::introspection;
//-------------------------------------------------------------------------------------------------------
// Delegates to ROS param access with the current NodeHandle
template <typename T>
bool ISmaccState::getParam(std::string param_name, T &param_storage)
{
    return nh.getParam(param_name, param_storage);
}
//-------------------------------------------------------------------------------------------------------

// Delegates to ROS param access with the current NodeHandle
template <typename T>
void ISmaccState::setParam(std::string param_name, T param_val)
{
    return nh.setParam(param_name, param_val);
}
//-------------------------------------------------------------------------------------------------------

//Delegates to ROS param access with the current NodeHandle
template <typename T>
bool ISmaccState::param(std::string param_name, T &param_val, const T &default_val) const
{
    return nh.param(param_name, param_val, default_val);
}
//-------------------------------------------------------------------------------------------------------

template <typename TOrthogonal, typename TBehavior, typename... Args>
void ISmaccState::configure(Args &&... args)
{
    std::string orthogonalkey = demangledTypeName<TOrthogonal>();
    ROS_INFO("Configuring orthogonal: %s", orthogonalkey.c_str());

    TOrthogonal *orthogonal = this->getOrthogonal<TOrthogonal>();
    if (orthogonal != nullptr)
    {
        auto clientBehavior = std::shared_ptr<TBehavior>(new TBehavior(args...));
        clientBehavior->template configureEventSourceTypes<TOrthogonal, TBehavior>();
        orthogonal->addClientBehavior(clientBehavior);
    }
    else
    {
        ROS_ERROR("Skipping client behavior creation. Orthogonal did not exist.");
    }
}
//-------------------------------------------------------------------------------------------------------

template <typename SmaccComponentType>
void ISmaccState::requiresComponent(SmaccComponentType *&storage)
{
    this->getStateMachine().requiresComponent(storage);
}
//-------------------------------------------------------------------------------------------------------

template <typename SmaccClientType>
void ISmaccState::requiresClient(SmaccClientType *&storage)
{
    storage = nullptr;
    auto &orthogonals = this->getStateMachine().getOrthogonals();
    for (auto &ortho : orthogonals)
    {
        ortho.second->requiresClient(storage);
        if (storage != nullptr)
            return;
    }

    ROS_ERROR("Client of type '%s' not found in any orthogonal of the current state machine. This may produce a segmentation fault if the returned reference is used.", demangleSymbol<SmaccClientType>().c_str());
}
//-------------------------------------------------------------------------------------------------------

template <typename T>
bool ISmaccState::getGlobalSMData(std::string name, T &ret)
{
    return this->getStateMachine().getGlobalSMData(name, ret);
}
//-------------------------------------------------------------------------------------------------------

// Store globally in this state machine. (By value parameter )
template <typename T>
void ISmaccState::setGlobalSMData(std::string name, T value)
{
    this->getStateMachine().setGlobalSMData(name, value);
}
//-------------------------------------------------------------------------------------------------------

template <typename TStateBehavior, typename TTriggerEvent, typename TEventList, typename... TEvArgs>
std::shared_ptr<TStateBehavior> ISmaccState::createStateBehavior(TEvArgs... args)
{
    auto sb = std::make_shared<TStateBehavior>(args...);
    TEventList *mock;
    sb->initialize(this, mock);
    sb->declarePostEvent(typelist<TTriggerEvent>());
    stateBehaviors_.push_back(sb);
    return sb;
}

template <typename TOrthogonal>
TOrthogonal* ISmaccState::getOrthogonal()
{
    return this->getStateMachine().getOrthogonal<TOrthogonal>();
}

// template <typename TStateBehavior, typename TTriggerEvent, typename... TEvArgs>
// std::shared_ptr<TStateBehavior> ISmaccState::createStateBehavior(TEvArgs... args)
// {
//     auto sb = std::make_shared<TStateBehavior>(std::forward(args...));
//     sb->initialize(this, typelist<TEvArgs...>());
//     sb->declarePostEvent(typelist<TTriggerEvent>());
//     stateBehaviors_.push_back(sb);

//     return sb;
// }
//-------------------------------------------------------------------------------------------------------

template <typename EventType>
void ISmaccState::postEvent(const EventType &ev)
{
    getStateMachine().postEvent(ev);
}
//-------------------------------------------------------------------------------------------------------

template <typename TransitionType>
void ISmaccState::notifyTransition()
{
    auto transitionType = TypeInfo::getTypeInfoFromType<TransitionType>();
    this->notifyTransitionFromTransitionTypeInfo(transitionType);
}

//-------------------------------------------------------------------------------------------------------------------

} // namespace smacc

#include <smacc/impl/smacc_state_behavior_impl.h>