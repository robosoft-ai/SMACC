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
struct EvConditionalTrue : sc::event<EvConditionalTrue<TSource, TObjectTag>>
{
};

//-----------------------------------------------------------------------
class SbConditional : public StateBehavior
{
private:
    std::map<const std::type_info *, bool> triggeredEvents;
    bool conditionFlag;

public:
    template <typename TEv>
    SbConditional(std::function<bool(TEv *)> conditionalFunction)
    {
        std::function<void(TEv *)> callback =
            [=](TEv *ev) {
                bool condition = conditionalFunction(ev);
                this->conditionFlag = condition;
            };

        this->createEventCallback(callback);
    }

    ~SbConditional();

    virtual bool triggers() override;
};
} // namespace state_behaviors
} // namespace smacc