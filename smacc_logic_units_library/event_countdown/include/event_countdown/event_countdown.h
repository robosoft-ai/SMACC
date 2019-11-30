#pragma once
#include <smacc/common.h>
#include <smacc/logic_units/logic_unit_base.h>
#include <map>
#include <typeinfo>
#include <boost/statechart/event.hpp>

namespace smacc
{

template <typename TSource, typename TObjectTag = empty_object_tag>
struct EvCountdownEnd : sc::event<EvCountdownEnd<TSource, TObjectTag>>
{
};

//-----------------------------------------------------------------------
class LuEventCountdown : public LogicUnit
{
private:
    std::map<const std::type_info *, bool> triggeredEvents;
    int eventCount_;

public:
    LuEventCountdown(int eventCount);

    virtual void onInitialized() override;

    virtual void onEventNotified(const std::type_info *eventType) override;

    virtual bool triggers() override;
};
} // namespace smacc