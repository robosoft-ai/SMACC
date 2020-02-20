#pragma once

#include <sm_packML/clients/cl_subscriber/cl_subscriber.h>
#include <smacc/smacc_orthogonal.h>

namespace sm_packML
{
using namespace sm_packML::cl_subscriber;

class OrSubscriber : public smacc::Orthogonal<OrSubscriber>
{
public:
    virtual void onInitialize() override
    {
        auto subscriber_client = this->createClient<ClSubscriber>();
        subscriber_client->initialize();
    }
};
} // namespace sm_packML