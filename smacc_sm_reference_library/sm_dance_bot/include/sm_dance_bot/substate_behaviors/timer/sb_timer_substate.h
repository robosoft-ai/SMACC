#pragma once

#include <smacc/smacc.h>
#include <sm_dance_bot/substate_behaviors/timer/timer_client.h>

namespace sm_dance_bot
{
class SbTimer : public smacc::SmaccSubStateBehavior
{
public:
  smacc::SmaccTimerClient *timerClient_;

  virtual void onEntry() override
  {
    this->requiresClient(timerClient_);

    timerClient_->onTimerTick.connect(
        [&](auto *ev) {
          auto ev2 = new smacc::EvTimer<SbTimer>();
          this->postEvent(ev2);
        });

    //this->propagatesEvents<EvTimer<SmaccTimer>>(timerClient_);
  }

  virtual void onExit() override
  {
  }
};
} // namespace sm_dance_bot
