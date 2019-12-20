#pragma once

#include <sm_three_some/clients/cl_client_2.h>
#include <smacc/smacc_orthogonal.h>

namespace sm_three_some
{
class OrOrthogonal2 : public smacc::Orthogonal
{
public:
    virtual void onInitialize() override
    {
        auto client2 = this->createClient<OrOrthogonal2, ClClient2>();
        client2->initialize();
    }
};
} // namespace sm_three_some