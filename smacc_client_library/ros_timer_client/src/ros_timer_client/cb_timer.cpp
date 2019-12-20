#include <ros_timer_client/client_behaviors/cb_timer.h>

namespace ros_timer_client
{
void CbTimer::onEntry()
{
    this->requiresClient(timerClient_);

    timerClient_->onTimerTick.connect(
        [&](auto *ev) {
            auto ev2 = new EvTimer<CbTimer>();
            this->postEvent(ev2);
        });

    //this->propagatesEvents<EvTimer<SmaccTimer>>(timerClient_);
}

void CbTimer::onExit()
{
}
} // namespace sm_dance_bot