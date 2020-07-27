#pragma once

#include <smacc/client_bases/smacc_publisher_client.h>
#include <std_msgs/String.h>

namespace sm_ridgeback_floor_coverage_static_1
{
namespace cl_string_publisher
{
class ClStringPublisher : public smacc::client_bases::SmaccPublisherClient
{
public:
    ClStringPublisher(std::string topicName)
        : smacc::client_bases::SmaccPublisherClient()
    {
        this->configure<std_msgs::String>(topicName);
    }
};
} // namespace cl_string_publisher
} // namespace sm_ridgeback_floor_coverage_static_1