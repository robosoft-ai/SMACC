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
struct EvConditionalTrue : sc::event<EvConditionalTrue<TSource, TObjectTag>>
{
};

//-----------------------------------------------------------------------
class LuConditional : public LogicUnit
{
private:
    std::map<const std::type_info *, bool> triggeredEvents;
    bool conditionFlag;

public:
    template <typename TEv>
    LuConditional(std::function<bool(TEv *)> conditionalFunction)
    {
        std::function<void(TEv *)> callback =
            [=](TEv *ev) {
                bool condition = conditionalFunction(ev);
                this->conditionFlag = condition;
            };

        this->createEventCallback(callback);
    }

    ~LuConditional();

    virtual bool triggers() override;
};
} // namespace logic_units
} // namespace smacc