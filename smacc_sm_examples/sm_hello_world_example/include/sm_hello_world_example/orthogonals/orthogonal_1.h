#pragma once

#include <sm_hello_world_example/clients/client_1.h>
#include <smacc/orthogonal.h>

namespace hello_world_example
{
class Orthogonal1 : public smacc::Orthogonal
{
public:
    virtual void onInitialize() override
    {
        auto client1 =  this->createClient<Client1>();
        client1->initialize();
    }
};
}