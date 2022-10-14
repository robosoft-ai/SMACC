
#pragma once
#include <smacc/client_bases/smacc_subscriber_client.h>
#include <std_msgs/Float32.h>

namespace sm_ferrari
{
namespace cl_subscriber
{
class ClSubscriber : public smacc::client_bases::SmaccSubscriberClient<std_msgs::Float32>
{
    public:
    ClSubscriber(std::string topicname):
        smacc::client_bases::SmaccSubscriberClient<std_msgs::Float32>(topicname)
    {
    }
};
} // namespace client_1
} // namespace sm_ferrari
