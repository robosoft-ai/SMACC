#pragma once

#include <smacc/orthogonal.h>
#include <sm_dance_bot/substate_behaviors/service_client/service3_client.h>

class Service3Orthogonal : public smacc::Orthogonal
{
public:
    virtual void onInitialize() override
    {
        auto* serviceClient = this->createClient<ServiceClient3>();
        serviceClient->serviceName_ = "/service_node3";
        serviceClient->initialize();
    }
};