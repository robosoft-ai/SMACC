#pragma once

#include <sm_three_some/clients/keyboard_client.h>
#include <smacc/smacc_orthogonal.h>
namespace sm_three_some
{
class OrKeyboard : public smacc::Orthogonal
{
public:
    virtual void onInitialize() override
    {
        auto keyboardClient = this->createClient<OrKeyboard, sm_three_some::KeyboardClient>();
        keyboardClient->topicName = "/keyboard_unicode";

        //keyboardClient.queueSize = 1;
        keyboardClient->initialize();
    }
};
} // namespace sm_three_some