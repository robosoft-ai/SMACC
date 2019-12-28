
#pragma once

#include <smacc/client_bases/smacc_publisher_client.h>
#include <std_msgs/String.h>

namespace sm_three_some
{
namespace updatable_publisher_client
{
class ClUpdatablePublisher : public smacc::SmaccPublisherClient<std_msgs::String>
{
public:
    ClUpdatablePublisher() : smacc::SmaccPublisherClient<std_msgs::String>()
    {
    }
};
} // namespace updatable_publisher_client
} // namespace sm_three_some