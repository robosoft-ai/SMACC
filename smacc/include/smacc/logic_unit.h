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
class ISMaccStateMachine;

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
            owner_->declareInputEvent<T>();
        }
    };

    template <typename TEventList>
    void initialize(ISmaccState *ownerState, TEventList *);

    virtual void onEventNotified(const std::type_info *eventType);

    // type based event callback
    template <typename T, typename TClass>
    void createEventCallback(void (TClass::*callback)(T *), TClass *object);

    // type based event callback
    template <typename T>
    void createEventCallback(std::function<void(T *)> callback);

    void update();

    //must returns true when the output event is triggered
    virtual bool triggers() = 0;

    template <typename TEv>
    void declareInputEvent();

    template <typename TEv>
    void declarePostEvent(smacc::introspection::typelist<TEv>);

private:
    friend ISmaccStateMachine;

    template <typename TEvent>
    void notifyEvent(TEvent *ev)
    {
        //the state machine uses this method to notify this logic unit some event happened.
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
};

} // namespace smacc
