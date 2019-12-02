#pragma once

#include <sm_three_some/substate_behaviors/keyboard/keyboard_client.h>
#include <smacc/smacc_orthogonal.h>
namespace sm_three_some
{
class KeyboardOrthogonal : public smacc::Orthogonal
{
public:
    virtual void onInitialize() override
    {
        auto keyboardClient = this->createClient<sm_three_some::KeyboardClient>();
        keyboardClient->topicName = "/keyboard_unicode";

        //keyboardClient.queueSize = 1;
        keyboardClient->initialize();
    }
};
} // namespace sm_three_some