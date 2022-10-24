#pragma once

#include <smacc/client_bases/smacc_action_client_base.h>
#include <sm_dance_bot_2/LEDControlAction.h>

namespace sm_dance_bot_2
{
namespace cl_led
{
class ClLED : public smacc::client_bases::SmaccActionClientBase<sm_dance_bot_2::LEDControlAction>
{

public:
    SMACC_ACTION_CLIENT_DEFINITION(sm_dance_bot_2::LEDControlAction);

    ClLED(std::string actionServerName);
    virtual std::string getName() const override;
    virtual ~ClLED();
};
} // namespace cl_led
}
