#include <ros_timer_client/cl_ros_timer.h>

namespace ros_timer_client
{

ClRosTimer::ClRosTimer(ros::Duration duration, bool oneshot)
{
    this->duration = duration;
    this->oneshot = oneshot;
}

ClRosTimer::~ClRosTimer()
{
    timer.stop();
}

void ClRosTimer::initialize()
{
    timer = nh_.createTimer(duration, boost::bind(&ClRosTimer::timerCallback, this, _1), oneshot);
}

void ClRosTimer::timerCallback(const ros::TimerEvent &timedata)
{
    //auto *event = new EvTimer<ClRosTimer>(this, timedata);
    auto *event = new EvTimer<ClRosTimer>();
    this->postEvent(event);
    onTimerTick(event);
}

} // namespace ros_timer_client
