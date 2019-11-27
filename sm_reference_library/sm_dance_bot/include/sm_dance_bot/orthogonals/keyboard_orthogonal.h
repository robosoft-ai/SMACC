#pragma once

#include <sm_dance_bot/substate_behaviors/keyboard/keyboard_client.h>
#include <smacc/smacc_orthogonal.h>
namespace sm_dancebot
{
class KeyboardOrthogonal : public smacc::Orthogonal
{
public:
    virtual void onInitialize() override
    {
        auto keyboardClient = this->createClient<sm_dancebot::KeyboardClient>();
        keyboardClient->topicName = "/keyboard_unicode";

        //keyboardClient.queueSize = 1;
        keyboardClient->initialize();
    }
};
} // namespace sm_dancebot