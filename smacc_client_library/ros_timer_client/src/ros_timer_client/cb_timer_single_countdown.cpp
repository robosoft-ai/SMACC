#include <ros_timer_client/client_behaviors/cb_timer_single_countdown.h>

namespace ros_timer_client
{

CbTimerSingleCountdown::CbTimerSingleCountdown(unsigned long triggerTickCount)
    : tickTriggerCount_(triggerTickCount),
      tickCounter_(0)
{
}

void CbTimerSingleCountdown::onClientTimerTickCallback()
{
    tickCounter_++;

    if (tickCounter_ % tickTriggerCount_ == 0)
    {
        onTimerTick_();
        postCountDownEvent_();
    }
}

void CbTimerSingleCountdown::onEntry()
{
    this->requiresClient(timerClient_);
    timerClient_->onTimerTick(&CbTimerSingleCountdown::onClientTimerTickCallback, this);
}

void CbTimerSingleCountdown::onExit()
{
}
} // namespace ros_timer_client