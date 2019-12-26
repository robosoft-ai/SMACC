#pragma once

#include <smacc/smacc.h>
#include <ros_timer_client/cl_ros_timer.h>

namespace ros_timer_client
{
class CbTimerSingleCountdown : public smacc::SmaccClientBehavior
{
public:
    CbTimerSingleCountdown(unsigned long triggerTickCount);

    virtual void onEntry() override;
    virtual void onExit() override;

    template <typename TObjectTag, typename TDerived>
    void configureEventSourceTypes()
    {
        this->postCountDownEvent_ = [=]() {
            this->template postEvent<EvTimer<TDerived, TObjectTag>>();
        };
    }

private:
    unsigned long tickCounter_;
    unsigned long tickTriggerCount_;

    ClRosTimer *timerClient_;
    std::function<void()> postCountDownEvent_;
};
} // namespace ros_timer_client