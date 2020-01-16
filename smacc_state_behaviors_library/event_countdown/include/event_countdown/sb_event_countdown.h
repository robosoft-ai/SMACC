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
struct EvCountdownEnd : sc::event<EvCountdownEnd<TSource, TObjectTag>>
{
};

//-----------------------------------------------------------------------
class SbEventCountdown : public StateBehavior
{
private:
    std::map<const std::type_info *, bool> triggeredEvents;
    int eventCount_;

public:
    SbEventCountdown(int eventCount);

    virtual void onInitialized() override;

    virtual void onEventNotified(const std::type_info *eventType) override;

    virtual bool triggers() override;
};
} // namespace state_behaviors
} // namespace smacc