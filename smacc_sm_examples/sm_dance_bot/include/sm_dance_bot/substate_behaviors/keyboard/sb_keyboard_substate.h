#pragma once

#include <sm_dance_bot/substate_behaviors/keyboard/keyboard_client.h>
#include <smacc/smacc_substate_behavior.h>

#include <std_msgs/UInt16.h>

namespace dance_bot
{
class SbKeyboard : public smacc::SmaccSubStateBehavior
{
public:
        KeyboardClient *keyboardClient_;
        boost::signals2::scoped_connection c_; 

        void onEntry()
        {
                this->requiresClient(keyboardClient_);

                c_ = this->keyboardClient_->OnKeyPress.connect(
                    [this](char character) {
                            this->OnKeyPress(character);
                    });
        }

        void OnKeyPress(char character)
        {
                if (character == 'a')
                        postKeyEvent<EvKeyPressA<SbKeyboard>>();
                else if (character == 'b')
                        postKeyEvent<EvKeyPressB<SbKeyboard>>();
                else if (character == 'c')
                        postKeyEvent<EvKeyPressC<SbKeyboard>>();
                else if (character == 'd')
                        postKeyEvent<EvKeyPressD<SbKeyboard>>();
                else if (character == 'e')
                        postKeyEvent<EvKeyPressE<SbKeyboard>>();
                else if (character == 'f')
                        postKeyEvent<EvKeyPressF<SbKeyboard>>();
                else if (character == 'g')
                        postKeyEvent<EvKeyPressG<SbKeyboard>>();
                else if (character == 'h')
                        postKeyEvent<EvKeyPressH<SbKeyboard>>();
                else if (character == 'y')
                        postKeyEvent<EvKeyPressI<SbKeyboard>>();
                else if (character == 'j')
                        postKeyEvent<EvKeyPressJ<SbKeyboard>>();
                else if (character == 'k')
                        postKeyEvent<EvKeyPressK<SbKeyboard>>();
                else if (character == 'l')
                        postKeyEvent<EvKeyPressL<SbKeyboard>>();
                else if (character == 'm')
                        postKeyEvent<EvKeyPressM<SbKeyboard>>();
                else if (character == 'n')
                        postKeyEvent<EvKeyPressN<SbKeyboard>>();
                else if (character == 'o')
                        postKeyEvent<EvKeyPressO<SbKeyboard>>();
                else if (character == 'p')
                        postKeyEvent<EvKeyPressP<SbKeyboard>>();
                else if (character == 'q')
                        postKeyEvent<EvKeyPressQ<SbKeyboard>>();
                else if (character == 'r')
                        postKeyEvent<EvKeyPressR<SbKeyboard>>();
                else if (character == 's')
                        postKeyEvent<EvKeyPressS<SbKeyboard>>();
                else if (character == 't')
                        postKeyEvent<EvKeyPressT<SbKeyboard>>();
                else if (character == 'u')
                        postKeyEvent<EvKeyPressU<SbKeyboard>>();
                else if (character == 'v')
                        postKeyEvent<EvKeyPressV<SbKeyboard>>();
                else if (character == 'w')
                        postKeyEvent<EvKeyPressW<SbKeyboard>>();
                else if (character == 'x')
                        postKeyEvent<EvKeyPressX<SbKeyboard>>();
                else if (character == 'y')
                        postKeyEvent<EvKeyPressY<SbKeyboard>>();
                else if (character == 'z')
                        postKeyEvent<EvKeyPressZ<SbKeyboard>>();
        }

        template <typename TEv>
        void postKeyEvent()
        {
                ROS_WARN("SbKeyboard %ld ev: %s", (long)(void*)this , demangleSymbol(typeid(TEv).name()).c_str());
                auto event = new TEv();
                this->postEvent(event);
        }
};

} // namespace smacc
