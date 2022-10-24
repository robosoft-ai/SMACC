#pragma once
#include <smacc/client_bases/smacc_subscriber_client.h>
#include <std_msgs/UInt16.h>

using namespace smacc::client_bases;

namespace sm_subscriber
{
class ClNumbersSubscription : public SmaccSubscriberClient<std_msgs::UInt16>
{
public:
  ClNumbersSubscription(std::string topicname) : SmaccSubscriberClient<std_msgs::UInt16>(topicname)
  {
  }
};
}  // namespace sm_subscriber
