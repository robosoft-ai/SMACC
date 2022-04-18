#include <smacc/smacc_updatable.h>

namespace smacc
{

ISmaccUpdatable::ISmaccUpdatable()
    : lastUpdate_(ros::Time::now())
{
}

ISmaccUpdatable::ISmaccUpdatable(ros::Duration duration)
    : lastUpdate_(ros::Time::now()),
      periodDuration_(duration)
{
}

void ISmaccUpdatable::setUpdatePeriod(ros::Duration duration)
{
    periodDuration_ = duration;
}

void ISmaccUpdatable::executeUpdate()
{
    bool update = true;
    if (periodDuration_)
    {
        auto now = ros::Time::now();
        auto ellapsed = now - this->lastUpdate_;
        update = ellapsed > *periodDuration_;
        if(update)
        {
            this->lastUpdate_ = now;
        }
    }

    if (update)
    {
        this->update();
    }
}
}