#include <smacc/logic_unit.h>

namespace smacc
{
template <typename TEv>
void LogicUnit::declarePostEvent(smacc::introspection::typelist<TEv>)
{
    this->postEventFn = [this]() {
        ROS_INFO_STREAM("[Logic Unit Base] postingfn posting event: " << demangleSymbol<TEv>());
        auto *ev = new TEv();
        this->ownerState->getStateMachine().postEvent(ev);
    };
}

template <typename TEv>
void LogicUnit::declareInputEvent()
{
}

template <typename TEventList>
void LogicUnit::initialize(ISmaccState *ownerState, TEventList *)
{
    this->ownerState = ownerState;

    using boost::mpl::_1;
    using wrappedList = typename boost::mpl::transform<TEventList, _1>::type;
    AddTEventType<TEventList> op(this);
    boost::mpl::for_each<wrappedList>(op);

    this->onInitialized();
}

template <typename T, typename TClass>
void LogicUnit::createEventCallback(void (TClass::*callback)(T *), TClass *object)
{
    const auto *eventtype = &typeid(T);
    this->eventCallbacks_[eventtype] = [=](void *msg) {
        T *evptr = (T *)msg;
        (object->*callback)(evptr);
    };
}

template <typename T>
void LogicUnit::createEventCallback(std::function<void(T *)> callback)
{
    const auto *eventtype = &typeid(T);
    this->eventCallbacks_[eventtype] = [=](void *msg) {
        T *evptr = (T *)msg;
        callback(evptr);
    };
}

} // namespace smacc