#pragma once

#include <sm_three_some/clients/cl_keyboard.h>
#include <smacc/smacc_orthogonal.h>
namespace sm_three_some
{
class OrKeyboard : public smacc::Orthogonal
{
public:
    virtual void onInitialize() override
    {
        auto clKeyboard = this->createClient<OrKeyboard, ClKeyboard>();
        clKeyboard->topicName = "/keyboard_unicode";

        //ClKeyboard.queueSize = 1;
        clKeyboard->initialize();
    }
};
} // namespace sm_three_some