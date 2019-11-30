#pragma once
#include <smacc/smacc_orthogonal.h>
#include <sm_dance_bot/substate_behaviors/timer/timer_client.h>

namespace sm_dance_bot
{
class TimerOrthogonal : public smacc::Orthogonal
{
public:
    virtual void onInitialize() override
    {
        auto *actionclient = this->createClient<smacc::SmaccTimerClient>(ros::Duration(0.5));
        actionclient->initialize();
    }
};
}