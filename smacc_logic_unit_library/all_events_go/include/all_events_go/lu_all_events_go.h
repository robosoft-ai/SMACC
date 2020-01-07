#pragma once
#include <smacc/common.h>
#include <smacc/logic_unit.h>
#include <map>
#include <typeinfo>
#include <boost/statechart/event.hpp>

namespace smacc
{

namespace logic_units
{
template <typename TSource, typename TObjectTag = empty_object_tag>
struct EvAllGo : sc::event<EvAllGo<TSource, TObjectTag>>
{
};

class LuAllEventsGo : public LogicUnit
{
    std::map<const std::type_info *, bool> triggeredEvents;

public:
    LuAllEventsGo()
    {
    }

    virtual void onInitialized() override;
    virtual void onEventNotified(const std::type_info *eventType) override;
    virtual bool triggers() override;
};
} // namespace logic_units
} // namespace smacc