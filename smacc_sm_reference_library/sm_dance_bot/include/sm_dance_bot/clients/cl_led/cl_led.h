#pragma once

#include <smacc/client_bases/smacc_action_client_base.h>
#include <sm_dance_bot/LEDControlAction.h>

namespace sm_dance_bot
{
namespace cl_led
{
class ClLED : public smacc::client_bases::SmaccActionClientBase<sm_dance_bot::LEDControlAction>
{
public:
    // for any action client you develop you need to call the ros action client type definition macro
    ACTION_DEFINITION(sm_dance_bot::LEDControlAction);

    ClLED();
    virtual std::string getName() const override;
    virtual ~ClLED();
};
} // namespace cl_led
}