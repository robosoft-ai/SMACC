#pragma once

#include <sm_dance_bot/substate_behaviors/publisher/client.h>
#include <smacc/smacc_substate_behavior.h>
#include <std_msgs/String.h>

namespace smacc
{
class SbStringPublisher : public smacc::SmaccSubStateBehavior
{
public:
    smacc::StringPublisherClient* publisherClient_;
    std::string msg_;

    SbStringPublisher(std::string msg)
    {
        msg_ = msg;
    }

    virtual void onEntry()
    {
        this->requiresComponent(publisherClient_);
        publisherClient_->initialize("/string_publisher_out");
    }

    virtual bool onExit()
    {
        std_msgs::String rosmsg;
        rosmsg.data =  msg_;
        publisherClient_->publish(rosmsg);
    }
};

} // namespace smacc
