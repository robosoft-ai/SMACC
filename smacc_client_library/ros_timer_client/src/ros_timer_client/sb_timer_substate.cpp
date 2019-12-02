#include <ros_timer_client/sb_timer_substate.h>

namespace ros_timer_client
{
void SbTimer::onEntry()
{
    this->requiresClient(timerClient_);

    timerClient_->onTimerTick.connect(
        [&](auto *ev) {
            auto ev2 = new EvTimer<SbTimer>();
            this->postEvent(ev2);
        });

    //this->propagatesEvents<EvTimer<SmaccTimer>>(timerClient_);
}

void SbTimer::onExit()
{
}
} // namespace sm_dance_bot