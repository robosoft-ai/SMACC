#pragma once

#include <ros_timer_client/cl_ros_timer.h>
#include <smacc/smacc_orthogonal.h>

namespace sm_calendar_week
{
class OrTimer : public smacc::Orthogonal<OrTimer>
{
public:
    virtual void onInitialize() override
    {
        auto actionclient = this->createClient<cl_ros_timer::ClRosTimer>(ros::Duration(1));
        actionclient->initialize();
    }
};
} // namespace sm_calendar_week
