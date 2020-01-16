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
    functor.fn = [](std::shared_ptr<smacc::StateBehavior> sb) {
        sb->addInputEvent<TEv>();
    };

    this->callbacks_.push_back(functor);
}

template <typename TEv>
void StateBehaviorHandler::setOutputEvent()
{
    CallbackFunctor functor;
    functor.fn = [](std::shared_ptr<smacc::StateBehavior> sb) {
        sb->setOutputEvent<TEv>();
    };

    this->callbacks_.push_back(functor);
}
} // namespace introspection

} // namespace smacc