#pragma once

#include <sm_three_some/clients/cl_client_1.h>
#include <smacc/smacc_orthogonal.h>

namespace sm_three_some
{
class OrOrthogonal1 : public smacc::Orthogonal
{
public:
    virtual void onInitialize() override
    {
        auto client1 = this->createClient<OrOrthogonal1, ClClient1>();
        client1->initialize();
    }
};
} // namespace sm_three_some