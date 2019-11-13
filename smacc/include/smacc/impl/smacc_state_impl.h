#pragma once
#include <smacc/smacc_state.h>
#include <smacc/orthogonal.h>
#include <smacc/smacc_substate_behavior.h>
#include <smacc/logic_units/logic_unit_base.h>
#include <smacc/string_type_walker.h>

namespace smacc
{
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

template <typename TOrthogonal>
void ISmaccState::configure(std::shared_ptr<SmaccSubStateBehavior> smaccBehavior)
{
    std::string orthogonalkey = demangledTypeName<TOrthogonal>();
    ROS_INFO("Configuring orthogonal: %s", orthogonalkey.c_str());

    std::shared_ptr<TOrthogonal> orthogonal;
    if (this->getStateMachine().getOrthogonal<TOrthogonal>(orthogonal))
    {
        orthogonal->setStateBehavior(smaccBehavior);
    }
    else
    {
        ROS_ERROR("Skipping substate behavior creation. Orthogonal did not exist.");
    }
}
//-------------------------------------------------------------------------------------------------------

template <typename SmaccComponentType>
void ISmaccState::requiresComponent(SmaccComponentType *&storage)
{
    this->getStateMachine().requiresComponent(storage);
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

template <typename TLUnit, typename TTriggerEvent, typename... TEvArgs>
std::shared_ptr<TLUnit> ISmaccState::createLogicUnit()
{
    auto lu = std::make_shared<TLUnit>();
    lu->initialize(this, typelist<TEvArgs...>());
    lu->declarePostEvent(typelist<TTriggerEvent>());
    logicUnits_.push_back(lu);

    return lu;
}
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
    auto transitionType = smacc::TypeInfo::getTypeInfoFromType<TransitionType>();
    this->notifyTransitionFromTransitionTypeInfo(transitionType);
}

//-------------------------------------------------------------------------------------------------------
template <typename TEv>
void LogicUnit::declarePostEvent(typelist<TEv>)
{
    this->postEventFn = [this]() {
        ROS_INFO_STREAM("[Logic Unit Base] postingfn posting event: " << demangleSymbol<TEv>());
        auto *ev = new TEv();
        this->ownerState->getStateMachine().postEvent(ev);
    };
}

//-------------------------------------------------------------------------------------------------------------------

} // namespace smacc