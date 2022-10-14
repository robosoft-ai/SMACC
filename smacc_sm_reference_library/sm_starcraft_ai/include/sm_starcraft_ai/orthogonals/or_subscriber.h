#pragma once

#include <sm_starcraft_ai/clients/cl_subscriber/cl_subscriber.h>
#include <smacc/smacc_orthogonal.h>

namespace sm_starcraft_ai
{
using namespace sm_starcraft_ai::cl_subscriber;

class OrSubscriber : public smacc::Orthogonal<OrSubscriber>
{
public:
    virtual void onInitialize() override
    {
        auto subscriber_client = this->createClient<ClSubscriber>();
        subscriber_client->initialize();
    }
};
} // namespace sm_starcraft_ai
