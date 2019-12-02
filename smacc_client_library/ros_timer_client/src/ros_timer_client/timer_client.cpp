#include <ros_timer_client/timer_client.h>

namespace ros_timer_client
{

SmaccTimerClient::SmaccTimerClient(ros::Duration duration, bool oneshot)
{
    this->duration = duration;
    this->oneshot = oneshot;
}

SmaccTimerClient::~SmaccTimerClient()
{
    timer.stop();
}

void SmaccTimerClient::initialize()
{
    timer = nh_.createTimer(duration, boost::bind(&SmaccTimerClient::timerCallback, this, _1), oneshot);
}

void SmaccTimerClient::timerCallback(const ros::TimerEvent &timedata)
{
    //auto *event = new EvTimer<SmaccTimerClient>(this, timedata);
    auto *event = new EvTimer<SmaccTimerClient>();
    this->postEvent(event);
    onTimerTick(event);
}

} // namespace ros_timer_client
