#pragma once

#include <smacc/smacc.h>
#include <ros_timer_client/cl_ros_timer.h>

namespace ros_timer_client
{

class CbTimerCountdownLoop : public smacc::SmaccClientBehavior
{
public:
    CbTimerCountdownLoop(unsigned long triggerTickCount);

    virtual void onEntry() override;
    virtual void onExit() override;

    template <typename TObjectTag, typename TDerived>
    void configureEventSourceTypes()
    {
        this->postCountDownEvent_ = [=]() {
            this->template postEvent<EvTimer<TDerived, TObjectTag>>();
        };
    }

    template <typename T>
    boost::signals2::connection onTimerTick(void (T::*callback)(), T *object)
    {
        return this->getStateMachine()->createSignalConnection(onTimerTick_, callback, object);
    }

private:
    unsigned long tickCounter_;
    unsigned long tickTriggerCount_;

    ClRosTimer *timerClient_;
    std::function<void()> postCountDownEvent_;
    smacc::SmaccSignal<void()> onTimerTick_;
    void onClientTimerTickCallback();
};
} // namespace ros_timer_client