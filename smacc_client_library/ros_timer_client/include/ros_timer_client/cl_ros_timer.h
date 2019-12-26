#pragma once

#include <smacc/smacc.h>
#include <boost/signals2.hpp>
#include <boost/optional/optional_io.hpp>

namespace ros_timer_client
{
template <typename TSource, typename TObjectTag>
struct EvTimer : sc::event<EvTimer<TSource, TObjectTag>>
{
    /*
    ClRosTimer *sender;
    ros::TimerEvent timedata;

    EvTimer(ClRosTimer *sender, const ros::TimerEvent &timedata)
    {
        this->sender = sender;
        this->timedata = timedata;
    }
    */
};

class ClRosTimer : public smacc::ISmaccClient
{
public:
    template <typename T>
    boost::signals2::connection onTimerTick(void (T::*callback)(), T *object)
    {
        return stateMachine_->createSignalConnection(onTimerTick_, callback, object);
    }

    template <typename TFunc>
    boost::signals2::connection onTimerTick(TFunc callback)
    {
        std::function<void()> callback1 = callback;
        return stateMachine_->createSignalConnection(onTimerTick_, callback1);
    }

    ClRosTimer(ros::Duration duration, bool oneshot = false);

    virtual ~ClRosTimer();

    virtual void initialize();

    template <typename TObjectTag, typename TDerived>
    void configureEventSourceTypes()
    {
        this->postTimerEvent_ = [=]() {
            this->postEvent<EvTimer<ClRosTimer, TObjectTag>>();
        };
    }

protected:
    ros::NodeHandle nh_;

    ros::Timer timer;
    ros::Duration duration;
    bool oneshot;

    void timerCallback(const ros::TimerEvent &timedata);
    std::function<void()> postTimerEvent_;
    smacc::SmaccSignal<void()> onTimerTick_;
};
} // namespace ros_timer_client
