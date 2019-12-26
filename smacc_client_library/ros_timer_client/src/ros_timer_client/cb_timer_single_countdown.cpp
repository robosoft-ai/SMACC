#include <ros_timer_client/client_behaviors/cb_timer_single_countdown.h>

namespace ros_timer_client
{

CbTimerSingleCountdown::CbTimerSingleCountdown(unsigned long triggerTickCount)
    : tickTriggerCount_(triggerTickCount),
      tickCounter_(0)
{
}

void CbTimerSingleCountdown::onEntry()
{
    this->requiresClient(timerClient_);

    timerClient_->onTimerTick(
        [=]() {
            tickCounter_++;

            if (tickCounter_ == tickTriggerCount_)
            {
                postCountDownEvent_();
            }
        });
}

    
void CbTimerSingleCountdown::onExit()
{
}
} // namespace ros_timer_client