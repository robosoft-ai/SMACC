#pragma once
#include <smacc/common.h>
#include <smacc/logic_units/logic_unit_base.h>
#include <map>
#include <typeinfo>
#include <boost/statechart/event.hpp>

namespace smacc
{

struct empty_object_tag
{
};

template <typename TSource, typename TObjectTag = empty_object_tag>
struct EvAll : sc::event<EvAll<TSource, TObjectTag>>
{
};

class LuAllEventsGo : public LogicUnit
{
    std::map<const std::type_info *, bool> triggeredEvents;

public:
    virtual void onInitialized() override;
    virtual void onEventNotified(const std::type_info *eventType) override;
    virtual bool triggers() override;
};
} // namespace smacc