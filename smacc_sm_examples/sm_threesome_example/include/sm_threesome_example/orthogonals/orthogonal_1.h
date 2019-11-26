#pragma once

#include <sm_threesome_example/clients/client_1.h>
#include <smacc/orthogonal.h>

namespace sm_threesome_example
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
} // namespace sm_threesome_example