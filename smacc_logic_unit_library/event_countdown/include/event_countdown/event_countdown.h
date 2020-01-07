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
template <typename TSource, typename TObjectTag = EmptyObjectTag>
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
} // namespace logic_units
} // namespace smacc