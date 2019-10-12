#pragma once

#include <functional>
#include <memory>
#include <vector>
#include <algorithm>

namespace smacc
{
class ISmaccState;

template <typename...>
struct typelist
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

    template <typename Head, typename... Tail>
    void initialize(ISmaccState *ownerState, typelist<Head, Tail...>)
    {
        eventTypes.push_back(&typeid(Head));
        initialize<Tail...>(ownerState, typelist<Tail...>());
    }

    template <typename Head>
    void initialize(ISmaccState *ownerState, typelist<Head>)
    {
        eventTypes.push_back(&typeid(Head));

        this->ownerState = ownerState;
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
