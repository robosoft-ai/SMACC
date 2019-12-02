#pragma once
#include <smacc/smacc_orthogonal.h>
#include <ros_timer_client/timer_client.h>

namespace sm_dance_bot
{
class TimerOrthogonal : public smacc::Orthogonal
{
public:
    virtual void onInitialize() override
    {
        auto *actionclient = this->createClient<ros_timer_client::SmaccTimerClient>(ros::Duration(0.5));
        actionclient->initialize();
    }
};
}