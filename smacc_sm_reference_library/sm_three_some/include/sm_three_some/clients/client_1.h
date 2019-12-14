
#pragma once
#include <smacc/client_bases/smacc_subscriber_client.h>
#include <boost/statechart/event.hpp>
#include <std_msgs/UInt16.h>

namespace sm_three_some
{
class Client1 : public smacc::SmaccSubscriberClient<std_msgs::UInt16>
{
};
} // namespace sm_three_some