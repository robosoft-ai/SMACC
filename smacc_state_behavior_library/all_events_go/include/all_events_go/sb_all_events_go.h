#pragma once
#include <smacc/common.h>
#include <smacc/smacc_state_behavior.h>
#include <map>
#include <typeinfo>
#include <boost/statechart/event.hpp>

namespace smacc
{

namespace state_behaviors
{
template <typename TSource, typename TObjectTag = EmptyObjectTag>
struct EvAllGo : sc::event<EvAllGo<TSource, TObjectTag>>
{
};

class SbAllEventsGo : public StateBehavior
{
    std::map<const std::type_info *, bool> triggeredEvents;

public:
    SbAllEventsGo()
    {
    }

    virtual void onInitialized() override;
    virtual void onEventNotified(const std::type_info *eventType) override;
    virtual bool triggers() override;
};
} // namespace state_behaviors
} // namespace smacc