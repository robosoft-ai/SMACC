#pragma once

#include <smacc/smacc.h>
#include <boost/signals2.hpp>
#include <boost/optional/optional_io.hpp>

namespace ros_timer_client
{
template <typename TSource>
struct EvTimer : sc::event<EvTimer<TSource>>
{
    /*
    SmaccTimerClient *sender;
    ros::TimerEvent timedata;

    EvTimer(SmaccTimerClient *sender, const ros::TimerEvent &timedata)
    {
        this->sender = sender;
        this->timedata = timedata;
    }
    */
};

class SmaccTimerClient : public smacc::ISmaccClient
{
public:
    boost::signals2::signal<void(EvTimer<SmaccTimerClient> *)> onTimerTick;
    boost::signals2::scoped_connection c_;

    SmaccTimerClient(ros::Duration duration, bool oneshot = false);

    virtual ~SmaccTimerClient();

    virtual void initialize();

protected:
    ros::NodeHandle nh_;

    ros::Timer timer;
    ros::Duration duration;
    bool oneshot;

    void timerCallback(const ros::TimerEvent &timedata);
};
} // namespace ros_timer_client
