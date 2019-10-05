#pragma once

#include <smacc/orthogonal.h>
#include <sm_dance_bot/substate_behaviors/publisher/string_publisher_client.h>

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