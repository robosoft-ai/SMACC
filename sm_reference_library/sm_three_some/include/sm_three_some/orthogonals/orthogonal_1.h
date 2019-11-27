#pragma once

#include <sm_three_some/clients/client_1.h>
#include <smacc/smacc_orthogonal.h>

namespace sm_three_some
{
class Orthogonal1 : public smacc::Orthogonal
{
public:
    virtual void onInitialize() override
    {
        auto client1 = this->createClient<Client1>();
        client1->initialize();
    }
};
} // namespace sm_three_some