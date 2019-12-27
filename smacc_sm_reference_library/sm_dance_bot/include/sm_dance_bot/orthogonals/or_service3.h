#pragma once

#include <smacc/smacc_orthogonal.h>
#include <sm_dance_bot/clients/service3_client/cl_service3.h>

namespace sm_dance_bot
{
class OrService3 : public smacc::Orthogonal
{
public:
    virtual void onInitialize() override
    {
        auto serviceClient = this->createClient<OrService3, ClService3>();
        serviceClient->serviceName_ = "/service_node3";
        serviceClient->initialize();
    }
};
} // namespace sm_dance_bot