#pragma once

#include <sm_dance_bot/client_behaviors/updatable_publisher/updatable_publisher_client.h>
#include <smacc/smacc_client_behavior.h>
#include <std_msgs/String.h>

namespace sm_dance_bot
{
class CbUpdatableStringPublisher : public smacc::SmaccClientBehavior,
                                   public smacc::ISmaccUpdatable
{
public:
    sm_dance_bot::UpdatableStringPublisherClient *publisherClient_;
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

} // namespace sm_dance_bot
