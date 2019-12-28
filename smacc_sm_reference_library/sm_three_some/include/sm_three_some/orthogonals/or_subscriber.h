#pragma once

#include <sm_three_some/clients/subscriber_client/cl_subscriber.h>
#include <smacc/smacc_orthogonal.h>

namespace sm_three_some
{
using namespace sm_three_some::subscriber_client;

class OrSubscriber : public smacc::Orthogonal
{
public:
    virtual void onInitialize() override
    {
        auto subscriber_client = this->createClient<OrSubscriber, ClSubscriber>();
        subscriber_client->initialize();
    }
};
} // namespace sm_three_some