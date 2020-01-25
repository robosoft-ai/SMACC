#pragma once
#include <smacc/smacc_orthogonal.h>
#include <ros_timer_client/cl_ros_timer.h>

namespace sm_dance_bot_3
{
class OrTimer : public smacc::Orthogonal<OrTimer>
{
public:
    virtual void onInitialize() override
    {
        auto actionclient = this->createClient<ros_timer_client::ClRosTimer>(ros::Duration(0.5));
        actionclient->initialize();
    }
};
}