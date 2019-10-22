#pragma once

#include <sm_dance_bot/substate_behaviors/keyboard/keyboard_client.h>
#include <smacc/orthogonal.h>

class KeyboardOrthogonal : public smacc::Orthogonal
{
public:
    virtual void onInitialize() override
    {
        auto keyboardClient =  this->createClient<dance_bot::KeyboardClient>();
        keyboardClient->topicName = "/keyboard_unicode";
        
        //keyboardClient.queueSize = 1;
        keyboardClient->initialize();
    }
};