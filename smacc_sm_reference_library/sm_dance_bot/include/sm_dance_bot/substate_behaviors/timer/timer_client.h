#pragma once

#include <smacc/smacc_client.h>
#include <boost/signals2.hpp>
#include <boost/optional/optional_io.hpp>

namespace smacc
{

class SmaccTimerClient;

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

    SmaccTimerClient(ros::Duration duration, bool oneshot = false)
    {
        this->duration = duration;
        this->oneshot = oneshot;
    }

    virtual ~SmaccTimerClient()
    {
        timer.stop();
    }

    virtual void initialize()
    {
        timer = nh_.createTimer(duration, boost::bind(&SmaccTimerClient::timerCallback, this, _1), oneshot);
    }

protected:
    ros::NodeHandle nh_;

    ros::Timer timer;
    ros::Duration duration;
    bool oneshot;

private:
    void timerCallback(const ros::TimerEvent &timedata)
    {
        //auto *event = new EvTimer<SmaccTimerClient>(this, timedata);
        auto *event = new EvTimer<SmaccTimerClient>();
        this->postEvent(event);
        onTimerTick(event);
    }
};
} // namespace smacc
