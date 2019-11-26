#pragma once

#include <sm_threesome_example/clients/client_2.h>
#include <smacc/orthogonal.h>

namespace sm_threesome_example
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
} // namespace sm_threesome_example