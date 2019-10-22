#pragma once

#include <sm_dance_bot/substate_behaviors/publisher/string_publisher_client.h>
#include <smacc/smacc_substate_behavior.h>
#include <std_msgs/String.h>

namespace dance_bot
{
class SbStringPublisher : public smacc::SmaccSubStateBehavior
{
public:
    dance_bot::StringPublisherClient* publisherClient_;
    std::string msg_;

    SbStringPublisher(std::string msg)
    {
        ROS_INFO_STREAM("Creating SbStringPublisher behavior with stored message: " << msg);
        msg_ = msg;
    }

    virtual void onEntry()
    {
        this->requiresClient(publisherClient_);
    }

    virtual void onExit() override
    {
        std_msgs::String rosmsg;
        rosmsg.data =  msg_;
        publisherClient_->publish(rosmsg);
    }
};

} // namespace smacc
