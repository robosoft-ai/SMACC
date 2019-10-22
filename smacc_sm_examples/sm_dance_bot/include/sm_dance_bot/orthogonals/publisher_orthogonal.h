#pragma once

#include <sm_dance_bot/substate_behaviors/publisher/string_publisher_client.h>
#include <smacc/orthogonal.h>

class PublisherOrthogonal : public smacc::Orthogonal
{
public:
    virtual void onInitialize() override
    {
        auto* publisherClient_ = this->createClient<dance_bot::StringPublisherClient>();
        publisherClient_->topicName = "/string_publisher_out";
        publisherClient_->initialize();
    }
};