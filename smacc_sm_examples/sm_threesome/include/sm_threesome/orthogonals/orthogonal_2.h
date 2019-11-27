#pragma once

#include <sm_threesome/clients/client_2.h>
#include <smacc/smacc_orthogonal.h>

namespace sm_threesome
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
} // namespace sm_threesome