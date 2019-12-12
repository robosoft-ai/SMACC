#pragma once

#include <sm_dance_bot/substate_behaviors/updatable_publisher/updatable_publisher_client.h>
#include <smacc/smacc_substate_behavior.h>
#include <std_msgs/String.h>

namespace sm_dance_bot
{
class SbUpdatableStringPublisher : public smacc::SmaccSubStateBehavior,
                                   public smacc::ISmaccUpdatable
{
public:
    sm_dance_bot::UpdatableStringPublisherClient *publisherClient_;
    SbUpdatableStringPublisher()
    {
    }

    virtual void onEntry()
    {
        this->requiresClient(publisherClient_);
        ROS_INFO("[SbUpdatableStringPublisher] hello world!");
    }

    virtual void onExit() override
    {
        
    }

    virtual void update()
    {
        ROS_INFO("[SbUpdatableStringPublisher] update here!");
    }
};

} // namespace sm_dance_bot
