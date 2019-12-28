
#pragma once
#include <smacc/client_bases/smacc_subscriber_client.h>
#include <std_msgs/UInt16.h>

namespace sm_three_some
{
namespace subscriber_client
{
class ClSubscriber : public smacc::SmaccSubscriberClient<std_msgs::UInt16>
{
};
} // namespace client_1
} // namespace sm_three_some