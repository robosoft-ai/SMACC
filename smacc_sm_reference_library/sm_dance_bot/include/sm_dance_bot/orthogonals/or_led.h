#pragma once
#include <smacc/smacc_orthogonal.h>
#include <sm_dance_bot/clients/cl_led/cl_led.h>

namespace sm_dance_bot
{
class OrLED : public smacc::Orthogonal
{
public:
    virtual void onInitialize() override
    {
        auto actionclient = this->createClient<OrLED, cl_led::ClLED>();
        actionclient->name_ = "led_action_server";
        actionclient->initialize();
    }
};
}