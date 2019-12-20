#pragma once

#include <sm_dance_bot/clients/cl_string_publisher.h>
#include <smacc/smacc_orthogonal.h>
namespace sm_dance_bot
{
class OrStringPublisher : public smacc::Orthogonal
{
public:
    virtual void onInitialize() override
    {
        auto publisherClient_ = this->createClient<OrStringPublisher, sm_dance_bot::ClStringPublisher>();
        publisherClient_->topicName = "/string_publisher_out";
        publisherClient_->initialize();
    }
};
} // namespace sm_dance_bot