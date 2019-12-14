#pragma once

#include <sm_three_some/clients/keyboard_client.h>
#include <smacc/smacc_substate_behavior.h>

#include <std_msgs/UInt16.h>

namespace sm_three_some
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

        std::function<void(char)> postEventKeyPress;

        template <typename TDerived, typename TObjectTag>
        void assignToOrthogonal()
        {
                postEventKeyPress = [=](char character) {
                        if (character == 'a')
                                postKeyEvent<EvKeyPressA<SbKeyboard, TObjectTag>>();
                        else if (character == 'b')
                                postKeyEvent<EvKeyPressB<SbKeyboard, TObjectTag>>();
                        else if (character == 'c')
                                postKeyEvent<EvKeyPressC<SbKeyboard, TObjectTag>>();
                        else if (character == 'd')
                                postKeyEvent<EvKeyPressD<SbKeyboard, TObjectTag>>();
                        else if (character == 'e')
                                postKeyEvent<EvKeyPressE<SbKeyboard, TObjectTag>>();
                        else if (character == 'f')
                                postKeyEvent<EvKeyPressF<SbKeyboard, TObjectTag>>();
                        else if (character == 'g')
                                postKeyEvent<EvKeyPressG<SbKeyboard, TObjectTag>>();
                        else if (character == 'h')
                                postKeyEvent<EvKeyPressH<SbKeyboard, TObjectTag>>();
                        else if (character == 'y')
                                postKeyEvent<EvKeyPressI<SbKeyboard, TObjectTag>>();
                        else if (character == 'j')
                                postKeyEvent<EvKeyPressJ<SbKeyboard, TObjectTag>>();
                        else if (character == 'k')
                                postKeyEvent<EvKeyPressK<SbKeyboard, TObjectTag>>();
                        else if (character == 'l')
                                postKeyEvent<EvKeyPressL<SbKeyboard, TObjectTag>>();
                        else if (character == 'm')
                                postKeyEvent<EvKeyPressM<SbKeyboard, TObjectTag>>();
                        else if (character == 'n')
                                postKeyEvent<EvKeyPressN<SbKeyboard, TObjectTag>>();
                        else if (character == 'o')
                                postKeyEvent<EvKeyPressO<SbKeyboard, TObjectTag>>();
                        else if (character == 'p')
                                postKeyEvent<EvKeyPressP<SbKeyboard, TObjectTag>>();
                        else if (character == 'q')
                                postKeyEvent<EvKeyPressQ<SbKeyboard, TObjectTag>>();
                        else if (character == 'r')
                                postKeyEvent<EvKeyPressR<SbKeyboard, TObjectTag>>();
                        else if (character == 's')
                                postKeyEvent<EvKeyPressS<SbKeyboard, TObjectTag>>();
                        else if (character == 't')
                                postKeyEvent<EvKeyPressT<SbKeyboard, TObjectTag>>();
                        else if (character == 'u')
                                postKeyEvent<EvKeyPressU<SbKeyboard, TObjectTag>>();
                        else if (character == 'v')
                                postKeyEvent<EvKeyPressV<SbKeyboard, TObjectTag>>();
                        else if (character == 'w')
                                postKeyEvent<EvKeyPressW<SbKeyboard, TObjectTag>>();
                        else if (character == 'x')
                                postKeyEvent<EvKeyPressX<SbKeyboard, TObjectTag>>();
                        else if (character == 'y')
                                postKeyEvent<EvKeyPressY<SbKeyboard, TObjectTag>>();
                        else if (character == 'z')
                                postKeyEvent<EvKeyPressZ<SbKeyboard, TObjectTag>>();
                };
        }

        void OnKeyPress(char character)
        {
                postEventKeyPress(character);
        }

        template <typename TEv>
        void postKeyEvent()
        {
                ROS_WARN("SbKeyboard %ld ev: %s", (long)(void *)this, smacc::demangleSymbol(typeid(TEv).name()).c_str());
                auto event = new TEv();
                this->postEvent(event);
        }
};

} // namespace sm_three_some
