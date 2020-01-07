#pragma once

#include <functional>
#include <memory>
#include <vector>
#include <algorithm>
#include <smacc/introspection/introspection.h>
#include <boost/statechart/event.hpp>
#include <map>

namespace smacc
{
class ISmaccState;

namespace logic_units
{
struct EmptyObjectTag
{
};
} // namespace logic_units

class LogicUnit
{
public:
    ISmaccState *ownerState;
    std::function<void()> postEventFn;
    std::vector<const std::type_info *> eventTypes;
    std::map<const std::type_info *, std::function<void(void *)>> eventCallbacks_;

    LogicUnit();

    virtual void onInitialized();

    template <typename TEventList>
    struct AddTEventType
    {
        LogicUnit *owner_;
        AddTEventType(LogicUnit *owner) : owner_(owner)
        {
        }

        template <typename T>
        void operator()(T)
        {
            owner_->eventTypes.push_back(&typeid(T));
        }
    };

    template <typename TEventList>
    void initialize(ISmaccState *ownerState, TEventList *)
    {
        this->ownerState = ownerState;

        using boost::mpl::_1;
        using wrappedList = typename boost::mpl::transform<TEventList, _1>::type;
        AddTEventType<TEventList> op(this);
        boost::mpl::for_each<wrappedList>(op);

        this->onInitialized();
    }

    template <typename TEvent>
    void notifyEvent(TEvent *ev)
    {
        auto tid = &(typeid(TEvent));
        if (std::find(eventTypes.begin(), eventTypes.end(), tid) != eventTypes.end())
        {
            this->onEventNotified(tid);
            this->update();

            if (eventCallbacks_.count(tid))
            {
                eventCallbacks_[tid]((void *)ev);
            }
        }
    }

    virtual void onEventNotified(const std::type_info *eventType);

    template <typename T, typename TClass>
    void createEventCallback(void (TClass::*callback)(T *), TClass *object)
    {
        const auto *eventtype = &typeid(T);
        this->eventCallbacks_[eventtype] = [=](void *msg) {
            T *evptr = (T *)msg;
            (object->*callback)(evptr);
        };
    }

    template <typename T>
    void createEventCallback(std::function<void(T *)> callback)
    {
        const auto *eventtype = &typeid(T);
        this->eventCallbacks_[eventtype] = [=](void *msg) {
            T *evptr = (T *)msg;
            callback(evptr);
        };
    }

    void update();

    virtual bool triggers() = 0;

    template <typename TEv>
    void declarePostEvent(smacc::introspection::typelist<TEv>);
};

} // namespace smacc
