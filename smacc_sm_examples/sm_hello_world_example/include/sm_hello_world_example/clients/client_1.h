
#pragma once
#include <smacc/interface_components/smacc_topic_subscriber.h>
#include <boost/statechart/event.hpp>
#include <std_msgs/UInt16.h>

namespace hello_world_example
{
class Client1 : public smacc::SmaccTopicSubscriberClient<Client1, std_msgs::UInt16>
{

};
}