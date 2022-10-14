
#pragma once
#include <smacc/client_bases/smacc_subscriber_client.h>
#include <std_msgs/UInt16.h>

namespace sm_respira_1
{
namespace cl_subscriber
{
class ClSubscriber : public smacc::client_bases::SmaccSubscriberClient<std_msgs::UInt16>
{
};
} // namespace client_1
} // namespace sm_respira_1
