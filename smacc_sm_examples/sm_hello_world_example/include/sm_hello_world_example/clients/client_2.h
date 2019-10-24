
#pragma once
#include <smacc/interface_components/smacc_topic_subscriber.h>
#include <boost/statechart/event.hpp>

namespace hello_world_example
{
class Client2 : public smacc::SmaccTopicSubscriberClient<Client2, std_msgs::UInt16>
{
};
}