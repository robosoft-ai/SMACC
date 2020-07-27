#pragma once

#include <smacc/client_bases/smacc_action_client_base.h>
#include <sm_ridgeback_floor_coverage_static_1/LEDControlAction.h>

namespace sm_ridgeback_floor_coverage_static_1
{
namespace cl_led
{
class ClLED : public smacc::client_bases::SmaccActionClientBase<sm_ridgeback_floor_coverage_static_1::LEDControlAction>
{

public:
    SMACC_ACTION_CLIENT_DEFINITION(sm_ridgeback_floor_coverage_static_1::LEDControlAction);

    ClLED(std::string actionServerName);
    virtual std::string getName() const override;
    virtual ~ClLED();
};
} // namespace cl_led
}