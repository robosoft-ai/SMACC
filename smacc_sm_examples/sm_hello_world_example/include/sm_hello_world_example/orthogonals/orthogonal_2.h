#pragma once

#include <sm_hello_world_example/clients/client_2.h>
#include <smacc/orthogonal.h>

namespace hello_world_example
{
class Orthogonal2 : public smacc::Orthogonal
{
public:
    virtual void onInitialize() override
    {
        auto client2 =  this->createClient<Client2>();
        client2->initialize();
    }
};
}