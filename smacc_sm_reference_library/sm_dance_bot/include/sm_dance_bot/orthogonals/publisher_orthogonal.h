#pragma once

#include <sm_dance_bot/client_behaviors/publisher/string_publisher_client.h>
#include <smacc/smacc_orthogonal.h>
namespace sm_dance_bot
{
class PublisherOrthogonal : public smacc::Orthogonal
{
public:
    virtual void onInitialize() override
    {
        auto publisherClient_ = this->createClient<PublisherOrthogonal, sm_dance_bot::StringPublisherClient>();
        publisherClient_->topicName = "/string_publisher_out";
        publisherClient_->initialize();
    }
};
} // namespace sm_dance_bot