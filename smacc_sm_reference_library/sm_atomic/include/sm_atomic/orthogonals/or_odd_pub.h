#pragma once

#include <sm_atomic/clients/cl_odd_pub/cl_odd_pub.h>
#include <smacc/smacc_orthogonal.h>

namespace sm_atomic
{
class OrOddPub : public smacc::Orthogonal<OrOddPub>
{
public:
    virtual void onInitialize() override
    {
        auto publisherClient_ = this->createClient<ClOddPub>("/atomic_out");
    }
};
} // namespace sm_atomic