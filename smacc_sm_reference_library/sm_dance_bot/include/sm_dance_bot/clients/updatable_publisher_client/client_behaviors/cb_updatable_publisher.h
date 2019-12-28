#pragma once

#include <sm_dance_bot/clients/updatable_publisher_client/cl_updatable_publisher.h>
#include <smacc/smacc_client_behavior.h>
#include <std_msgs/String.h>

namespace sm_dance_bot
{
namespace ros_publisher_client
{
class CbUpdatableStringPublisher : public smacc::SmaccClientBehavior,
                                   public smacc::ISmaccUpdatable
{
public:
    ClUpdatableStringPublisher *publisherClient_;
    CbUpdatableStringPublisher()
    {
    }

    virtual void onEntry()
    {
        this->requiresClient(publisherClient_);
        ROS_INFO("[CbUpdatableStringPublisher] hello world!");
    }

    virtual void onExit() override
    {
    }

    virtual void update()
    {
        ROS_INFO("[CbUpdatableStringPublisher] update here!");
    }
};
} // namespace ros_publisher_client
} // namespace sm_dance_bot
