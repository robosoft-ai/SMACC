#pragma once
#include <smacc/smacc_orthogonal.h>
#include <sm_ridgeback_floor_coverage_dynamic_1/clients/cl_led/cl_led.h>

namespace sm_ridgeback_floor_coverage_dynamic_1
{
class OrLED : public smacc::Orthogonal<OrLED>
{
public:
    virtual void onInitialize() override
    {
        auto actionclient = this->createClient<cl_led::ClLED>("led_action_server");
        actionclient->initialize();
    }
};
} // namespace sm_ridgeback_floor_coverage_dynamic_1