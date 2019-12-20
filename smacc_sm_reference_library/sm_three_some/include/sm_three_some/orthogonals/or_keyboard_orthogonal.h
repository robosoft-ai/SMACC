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
        auto ClKeyboard = this->createClient<OrKeyboard, sm_three_some::ClKeyboard>();
        ClKeyboard->topicName = "/keyboard_unicode";

        //ClKeyboard.queueSize = 1;
        ClKeyboard->initialize();
    }
};
} // namespace sm_three_some