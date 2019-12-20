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
    boost::signals2::signal<void(EvTimer<ClRosTimer> *)> onTimerTick;
    boost::signals2::scoped_connection c_;

    ClRosTimer(ros::Duration duration, bool oneshot = false);

    virtual ~ClRosTimer();

    virtual void initialize();

protected:
    ros::NodeHandle nh_;

    ros::Timer timer;
    ros::Duration duration;
    bool oneshot;

    void timerCallback(const ros::TimerEvent &timedata);
};
} // namespace ros_timer_client
