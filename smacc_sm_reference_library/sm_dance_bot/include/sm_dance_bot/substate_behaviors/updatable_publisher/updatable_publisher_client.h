#pragma once

#include <smacc/client_bases/smacc_publisher_client.h>
#include <std_msgs/String.h>

namespace sm_dance_bot
{
class UpdatableStringPublisherClient : public smacc::SmaccPublisherClient<std_msgs::String>,
                              public smacc::ISmaccUpdatable
{
public:
    UpdatableStringPublisherClient() 
            : smacc::SmaccPublisherClient<std_msgs::String>()
    {
    }

    virtual void update()
    {
        ROS_INFO("[UpdatableStringPublisherClient] update here!");
    }
};
} // namespace sm_dance_bot