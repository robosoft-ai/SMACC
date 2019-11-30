#pragma once

#include <functional>
#include <memory>
#include <vector>
#include <algorithm>
#include <smacc/introspection/introspection.h>

namespace smacc
{
class ISmaccState;

struct empty_object_tag
{
};

class LogicUnit
{
public:
    ISmaccState *ownerState;
    std::function<void()> postEventFn;
    std::vector<const std::type_info *> eventTypes;

    LogicUnit();

    virtual void onInitialized();

    template<typename TEventList>
    struct AddTEventType
    {
        LogicUnit* owner_;
        AddTEventType(LogicUnit* owner):
            owner_(owner)
        {

        }
        
        template <typename T>
        void operator()(T)
        {
            owner_->eventTypes.push_back(&typeid(T));
        }
    };

    template <typename TEventList>
    void initialize(ISmaccState *ownerState, TEventList*)
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
        }
    }

    virtual void onEventNotified(const std::type_info *eventType);

    void update();

    virtual bool triggers() = 0;

    template <typename TEv>
    void declarePostEvent(typelist<TEv>);
};

} // namespace smacc
