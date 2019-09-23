#pragma once

#include <smacc/interface_components/smacc_topic_publisher.h>
#include <smacc/smacc_substate_behavior.h>
#include <std_msgs/String.h>

namespace smacc
{

class SbPublisher : public smacc::SmaccSubStateBehavior
{
public:
    smacc::SmaccTopicPublisherClient<std_msgs::String> *publisherClient_;

    virtual void onEntry()
    {
        this->requiresComponent(publisherClient_);
        publisherClient_->initialize("/waypoint_out");
    }

    virtual bool onExit()
    {
        std_msgs::String msg;
        msg.data = "Navigate to Waypoint 1 finished";
        publisherClient_->publish(msg);
    }
};

} // namespace smacc
