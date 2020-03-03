#pragma once

#include <keyboard_client/cl_keyboard.h>
#include <smacc/smacc_orthogonal.h>
namespace sm_mtc_pour
{

class OrKeyboard : public smacc::Orthogonal<OrKeyboard>
{
public:
    virtual void onInitialize() override
    {
        auto clKeyboard = this->createClient<cl_keyboard::ClKeyboard>();
        
        //ClKeyboard.queueSize = 1;
        clKeyboard->initialize();
    }
};
} // namespace sm_mtc_pour