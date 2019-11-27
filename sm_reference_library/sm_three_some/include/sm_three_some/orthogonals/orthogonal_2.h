#pragma once

#include <sm_three_some/clients/client_2.h>
#include <smacc/smacc_orthogonal.h>

namespace sm_three_some
{
class Orthogonal2 : public smacc::Orthogonal
{
public:
    virtual void onInitialize() override
    {
        auto client2 = this->createClient<Client2>();
        client2->initialize();
    }
};
} // namespace sm_three_some