/*****************************************************************************************************************
 * ReelRobotix Inc. - Software License Agreement      Copyright (c) 2018
 * 	 Authors: Pablo Inigo Blasco, Brett Aldrich
 *
 ******************************************************************************************************************/

#pragma once
#include <smacc/smacc_state_behavior.h>
#include <smacc/introspection/introspection.h>

namespace smacc
{
template <typename TEv>
void StateBehavior::setOutputEvent()
{
    this->postEventFn = [this]() {
        ROS_INFO_STREAM("[State Behavior Base] postingfn posting event: " << demangleSymbol<TEv>());
        auto *ev = new TEv();
        this->ownerState->getStateMachine().postEvent(ev);
    };
}

template <typename TEv>
void StateBehavior::addInputEvent()
{
    this->eventTypes.push_back(&typeid(TEv));
}

template <typename T, typename TClass>
void StateBehavior::createEventCallback(void (TClass::*callback)(T *), TClass *object)
{
    const auto *eventtype = &typeid(T);
    this->eventCallbacks_[eventtype] = [=](void *msg) {
        T *evptr = (T *)msg;
        (object->*callback)(evptr);
    };
}

template <typename T>
void StateBehavior::createEventCallback(std::function<void(T *)> callback)
{
    const auto *eventtype = &typeid(T);
    this->eventCallbacks_[eventtype] = [=](void *msg) {
        T *evptr = (T *)msg;
        callback(evptr);
    };
}

namespace introspection
{

template <typename TEv>
void StateBehaviorHandler::addInputEvent()
{
    CallbackFunctor functor;
    functor.fn = [=](std::shared_ptr<smacc::StateBehavior> sb) {
        ROS_INFO("[%s] State Behavior adding input event: %s", demangleType(sbInfo_->stateBehaviorType).c_str(), demangledTypeName<TEv>().c_str());
        sb->addInputEvent<TEv>();
    };

    this->callbacks_.push_back(functor);

    auto evtype = TypeInfo::getFromStdTypeInfo(typeid(TEv));
    auto evinfo = std::make_shared<SmaccEventInfo>(evtype);
    EventLabel<TEv>(evinfo->label);

    sbInfo_->sourceEventTypes.push_back(evinfo);
}

template <typename TEv>
void StateBehaviorHandler::setOutputEvent()
{
    CallbackFunctor functor;
    functor.fn = [=](std::shared_ptr<smacc::StateBehavior> sb) {
        ROS_INFO("[%s] State Behavior setting output event: %s", demangleType(sbInfo_->stateBehaviorType).c_str(), demangledTypeName<TEv>().c_str());
        sb->setOutputEvent<TEv>();
    };

    this->callbacks_.push_back(functor);
}
} // namespace introspection

} // namespace smacc