#pragma once
#include <smacc/logic_units/logic_unit_base.h>
#include <map>
#include <typeinfo>

namespace smacc
{
class LuAll : public LogicUnit
{
    std::map<const std::type_info*, bool> triggeredEvents;

    public:
    
    virtual void onInitialized() override;
    virtual void onEventNotified(const std::type_info* eventType) override;
    virtual bool triggers() override;
};
}