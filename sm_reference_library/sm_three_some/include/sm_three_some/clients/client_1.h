
#pragma once
#include <smacc/client_bases/smacc_topic_subscriber.h>
#include <boost/statechart/event.hpp>
#include <std_msgs/UInt16.h>

namespace sm_three_some
{
class Client1 : public smacc::SmaccTopicSubscriberClient<Client1, std_msgs::UInt16>
{
};
} // namespace sm_three_some