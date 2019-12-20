#pragma once

#include <smacc/smacc_state_machine.h>
#include <smacc/smacc_state.h>
#include <smacc/smacc_orthogonal.h>
#include <smacc/smacc_signal_detector.h>
#include <smacc/introspection/introspection.h>
#include <smacc_msgs/SmaccStatus.h>
#include <sstream>
#include <smacc/logic_units/logic_unit_base.h>

namespace smacc
{
template <typename TOrthogonal>
bool ISmaccStateMachine::getOrthogonal(std::shared_ptr<TOrthogonal> &storage)
{
    std::lock_guard<std::recursive_mutex> lock(m_mutex_);

    std::string orthogonalkey = demangledTypeName<TOrthogonal>();
    std::shared_ptr<TOrthogonal> ret;

    auto it = orthogonals_.find(orthogonalkey);

    if (it != orthogonals_.end())
    {
        ROS_INFO("%s resource is required. Found resource in cache.", orthogonalkey.c_str());
        ret = dynamic_pointer_cast<TOrthogonal>(it->second);
        storage = ret;
        return true;
    }
    else
    {
        std::stringstream ss;
        ss << "Orthogonal not found " << orthogonalkey.c_str() << std::endl;
        ss << "The existing orthogonals are the following: " << std::endl;
        for (auto &orthogonal : orthogonals_)
        {
            ss << " - " << orthogonal.first << std::endl;
        }

        ROS_WARN_STREAM(ss.str());

        return false;
    }
}

//-------------------------------------------------------------------------------------------------------
template <typename TOrthogonal>
void ISmaccStateMachine::createOrthogonal()
{
    this->lockStateMachine("create orthogonal");
    std::string orthogonalkey = demangledTypeName<TOrthogonal>();

    if (orthogonals_.count(orthogonalkey) == 0)
    {
        auto ret = std::make_shared<TOrthogonal>();
        orthogonals_[orthogonalkey] = dynamic_pointer_cast<smacc::Orthogonal>(ret);

        ret->setStateMachine(this);

        ROS_INFO("%s Orthogonal is created", orthogonalkey.c_str());
    }
    else
    {
        ROS_WARN_STREAM("There were already one existing orthogonal of type " << orthogonalkey.c_str() << ". Skipping creation orthogonal request. ");
        std::stringstream ss;
        ss << "The existing orthogonals are the following: " << std::endl;
        for (auto &orthogonal : orthogonals_)
        {
            ss << " - " << orthogonal.first << std::endl;
        }
        ROS_WARN_STREAM(ss.str());
    }
    this->unlockStateMachine("create orthogonal");
}

//-------------------------------------------------------------------------------------------------------
template <typename SmaccComponentType>
void ISmaccStateMachine::requiresComponent(SmaccComponentType *&storage, bool verbose)
{
    ROS_INFO("component %s is required", demangleSymbol(typeid(SmaccComponentType).name()).c_str());
    std::lock_guard<std::recursive_mutex> lock(m_mutex_);

    std::string pluginkey = demangledTypeName<SmaccComponentType>();
    SmaccComponentType *ret;

    auto it = plugins_.find(pluginkey);

    if (it == plugins_.end())
    {
        ROS_INFO("%s smacc component is required. Creating a new instance.", pluginkey.c_str());

        ret = new SmaccComponentType();
        ret->setStateMachine(this);
        plugins_[pluginkey] = static_cast<smacc::ISmaccComponent *>(ret);
        ROS_INFO("%s resource is required. Done.", pluginkey.c_str());
    }
    else
    {
        ROS_INFO("%s resource is required. Found resource in cache.", pluginkey.c_str());
        ret = dynamic_cast<SmaccComponentType *>(it->second);
    }

    storage = ret;
}
//-------------------------------------------------------------------------------------------------------
template <typename EventType>
void ISmaccStateMachine::postEvent(EventType *ev)
{
    std::lock_guard<std::recursive_mutex> lock(m_mutex_);

    // when a postting event is requested by any component, client, or substate behavior
    // we reach this place. Now, we propagate the events to all the state logic units to generate
    // some more events

    auto currentstate = currentState_;
    if (currentstate != nullptr)
    {
        ROS_DEBUG_STREAM("EVENT: " << demangleSymbol<EventType>());
        for (auto &lu : currentstate->getLogicUnits())
        {
            lu->notifyEvent(ev);
        }
    }

    this->signalDetector_->postEvent(ev);
}

template <typename T>
bool ISmaccStateMachine::getGlobalSMData(std::string name, T &ret)
{
    std::lock_guard<std::recursive_mutex> lock(m_mutex_);
    //ROS_WARN("get SM Data lock acquire");
    bool success = false;

    if (!globalData_.count(name))
    {
        //ROS_WARN("get SM Data - data do not exist");
        success = false;
    }
    else
    {
        //ROS_WARN("get SM DAta -data exist. accessing");
        try
        {
            auto &v = globalData_[name];

            //ROS_WARN("get SM DAta -data exist. any cast");
            ret = boost::any_cast<T>(v.second);
            success = true;
            //ROS_WARN("get SM DAta -data exist. success");
        }
        catch (boost::bad_any_cast &ex)
        {
            ROS_ERROR("bad any cast: %s", ex.what());
            success = false;
        }
    }

    //ROS_WARN("get SM Data lock release");
    return success;
}

template <typename T>
void ISmaccStateMachine::setGlobalSMData(std::string name, T value)
{
    {
        std::lock_guard<std::recursive_mutex> lock(m_mutex_);
        //ROS_WARN("set SM Data lock acquire");

        globalData_[name] = {
            [this, name]() {
                std::stringstream ss;
                auto val = any_cast<T>(globalData_[name].second);
                ss << val;
                return ss.str();
            },
            value};
    }

    this->updateStatusMessage();
}

template <typename StateField, typename BehaviorType>
void ISmaccStateMachine::mapBehavior()
{
    std::string stateFieldName = demangleSymbol(typeid(StateField).name());
    std::string behaviorType = demangleSymbol(typeid(BehaviorType).name());
    ROS_INFO("Mapping state field '%s' to stateBehavior '%s'", stateFieldName.c_str(), behaviorType.c_str());
    SmaccSubStateBehavior *globalreference;
    if (!this->getGlobalSMData(stateFieldName, globalreference))
    {
        // Using the requires component approach, we force a unique existence
        // of this component
        BehaviorType *behavior;
        this->requiresComponent(behavior);
        globalreference = dynamic_cast<SmaccSubStateBehavior *>(behavior);

        this->setGlobalSMData(stateFieldName, globalreference);
    }
}

template <typename TSmaccSignal, typename TMemberFunctionPrototype, typename TSmaccObjectType>
boost::signals2::connection ISmaccStateMachine::createSignalConnection(TSmaccSignal &signal, TMemberFunctionPrototype callback, TSmaccObjectType *object)
{
    static_assert(std::is_base_of<ISmaccState, TSmaccObjectType>::value || std::is_base_of<SmaccSubStateBehavior, TSmaccObjectType>::value || std::is_base_of<LogicUnit, TSmaccObjectType>::value || std::is_base_of<ISmaccComponent, TSmaccObjectType>::value, "Only are accepted smacc types as subscribers for smacc signals");

    boost::signals2::connection connection = signal.connect([=](auto msg) { return (object->*callback)(msg); });

    if(std::is_base_of<ISmaccComponent, TSmaccObjectType>::value)
    {

    }
    else // state life-time objects
    {
        ROS_WARN("[StateMachine] life-time constrained smacc signal subscription created");
        stateCallbackConnections.push_back(connection);
    }
    return connection;
}

template <typename TSmaccSignal, typename TMemberFunctionPrototype>
boost::signals2::connection ISmaccStateMachine::createSignalConnection(TSmaccSignal &signal, TMemberFunctionPrototype callback)
{
    return signal.connect(callback);
    // return signal;
}

template <typename T>
bool ISmaccStateMachine::getParam(std::string param_name, T &param_storage)
{
    return nh_.getParam(param_name, param_storage);
}

// Delegates to ROS param access with the current NodeHandle
template <typename T>
void ISmaccStateMachine::setParam(std::string param_name, T param_val)
{
    return nh_.setParam(param_name, param_val);
}

// Delegates to ROS param access with the current NodeHandle
template <typename T>
bool ISmaccStateMachine::param(std::string param_name, T &param_val, const T &default_val) const
{
    return nh_.param(param_name, param_val, default_val);
}

template <typename StateType>
void ISmaccStateMachine::notifyOnStateEntryStart(StateType *state)
{
std:
    lock_guard<std::recursive_mutex> lock(m_mutex_);

    ROS_INFO_STREAM("Notification State Entry, orthogonals:" << this->orthogonals_.size() << ", new state " << state);

    stateSeqCounter_++;
    currentState_ = state;
    currentStateInfo_ = info_->getState<StateType>();
}

template <typename StateType>
void ISmaccStateMachine::notifyOnStateEntryEnd(StateType *state)
{
    for (auto pair : this->orthogonals_)
    {
        ROS_INFO("ortho onentry: %s", pair.second->getName().c_str());
        auto &orthogonal = pair.second;
        orthogonal->onEntry();
    }

    this->updateStatusMessage();
}

template <typename StateType>
void ISmaccStateMachine::notifyOnStateExit(StateType *state)
{
    ROS_INFO_STREAM("Notification State Exit: leaving state" << state);
    for (auto pair : this->orthogonals_)
    {
        auto &orthogonal = pair.second;
        orthogonal->onExit();
    }

    for (auto &conn : this->stateCallbackConnections)
    {
        ROS_WARN_STREAM("[StateMachine] Disconnecting scoped-lifetime SmaccSignal subscription");
        conn.disconnect();
    }

    this->stateCallbackConnections.clear();

    currentState_ = nullptr;
}
} // namespace smacc