
#pragma once
#include <smacc/client_bases/smacc_subscriber_client.h>
#include <boost/statechart/event.hpp>
#include <std_msgs/UInt16.h>

namespace sm_three_some
{
namespace client_1
{
class ClClient1 : public smacc::SmaccSubscriberClient<std_msgs::UInt16>
{
};
} // namespace client_1
} // namespace sm_three_some