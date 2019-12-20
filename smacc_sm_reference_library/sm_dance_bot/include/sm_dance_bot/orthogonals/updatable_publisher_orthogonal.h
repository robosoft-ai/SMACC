#pragma once

#include <sm_dance_bot/client_behaviors/updatable_publisher/updatable_publisher_client.h>
#include <smacc/smacc_orthogonal.h>

namespace sm_dance_bot
{
class UpdatablePublisherOrthogonal : public smacc::Orthogonal
{
public:
    virtual void onInitialize() override
    {
        auto publisherClient_ = this->createClient<UpdatablePublisherOrthogonal, sm_dance_bot::UpdatableStringPublisherClient>();
        publisherClient_->topicName = "/updatable_string_publisher_out";
        publisherClient_->initialize();
    }
};
} // namespace sm_dance_bot