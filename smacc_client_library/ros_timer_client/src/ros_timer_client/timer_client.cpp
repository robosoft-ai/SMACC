#include <ros_timer_client/cl_ros_timer.h>

namespace cl_ros_timer
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
    if (!onTimerTick_.empty())
    {
        this->onTimerTick_();
    }
    postTimerEvent_();
}

} // namespace cl_ros_timer
