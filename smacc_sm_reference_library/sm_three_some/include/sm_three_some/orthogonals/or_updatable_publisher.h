#pragma once

#include <sm_three_some/clients/updatable_publisher_client/cl_updatable_publisher.h>
#include <smacc/smacc_orthogonal.h>

namespace sm_three_some
{
using namespace sm_three_some::updatable_publisher_client;

class OrUpdatablePublisher : public smacc::Orthogonal
{
public:
    virtual void onInitialize() override
    {
        auto updatable_publisher_client = this->createClient<OrUpdatablePublisher, ClUpdatablePublisher>();
        updatable_publisher_client->initialize();
    }
};
} // namespace sm_three_some