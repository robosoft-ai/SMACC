#include <ros_timer_client/client_behaviors/cb_timer_repeat_countdown.h>

namespace ros_timer_client
{

CbTimerRepeatCountdown::CbTimerRepeatCountdown(unsigned long triggerTickCount)
    : tickTriggerCount_(triggerTickCount),
      tickCounter_(0)
{
}

void CbTimerRepeatCountdown::onEntry()
{
    this->requiresClient(timerClient_);

    timerClient_->onTimerTick(
        [=]() {
            tickCounter_++;

            if (tickCounter_ % tickTriggerCount_ == 0)
            {
                postCountDownEvent_();
            }
        });
}

void CbTimerRepeatCountdown::onExit()
{
}
} // namespace ros_timer_client