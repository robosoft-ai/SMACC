#pragma once
#include <smacc/common.h>
#include <smacc/smacc_state_reactor.h>
#include <map>
#include <typeinfo>
#include <boost/statechart/event.hpp>

namespace smacc
{
namespace state_reactors
{
template <typename TSource, typename TObjectTag = EmptyObjectTag>
struct EvConditionalTrue : sc::event<EvConditionalTrue<TSource, TObjectTag>>
{
};

//-----------------------------------------------------------------------
class SrConditional : public StateReactor
{
private:
    std::map<const std::type_info *, bool> triggeredEvents;
    bool conditionFlag;

public:
    template <typename TEv>
    SrConditional(std::function<bool(TEv *)> conditionalFunction)
    {
        std::function<void(TEv *)> callback =
            [=](TEv *ev) {
                bool condition = conditionalFunction(ev);
                this->conditionFlag = condition;
            };

        this->createEventCallback(callback);
    }

    ~SrConditional();

    virtual bool triggers() override;
};
} // namespace state_reactors
} // namespace smacc