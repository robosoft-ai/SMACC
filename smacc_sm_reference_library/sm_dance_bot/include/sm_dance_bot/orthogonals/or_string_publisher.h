#pragma once

#include <sm_dance_bot/clients/cl_string_publisher/cl_string_publisher.h>
#include <smacc/smacc_orthogonal.h>
namespace sm_dance_bot
{
class OrStringPublisher : public smacc::Orthogonal
{
public:
    virtual void onInitialize() override
    {
        this->createClient<OrStringPublisher, ClStringPublisher>("/string_publisher_out");
    }
};
} // namespace sm_dance_bot