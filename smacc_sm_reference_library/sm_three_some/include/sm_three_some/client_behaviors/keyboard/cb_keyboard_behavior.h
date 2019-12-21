#pragma once

#include <sm_three_some/clients/cl_keyboard.h>
#include <smacc/smacc_client_behavior.h>

#include <std_msgs/UInt16.h>

namespace sm_three_some
{
class CbKeyboard : public smacc::SmaccClientBehavior
{
public:
        ClKeyboard *ClKeyboard_;
        boost::signals2::scoped_connection c_;

        void onEntry()
        {
                this->requiresClient(ClKeyboard_);

                c_ = this->ClKeyboard_->OnKeyPress.connect(
                    [this](char character) {
                            this->OnKeyPress(character);
                    });
        }

        std::function<void(char)> postEventKeyPress;

        template <typename TDerived, typename TObjectTag>
        void configureEventSourceTypes()
        {
                postEventKeyPress = [=](char character) {
                        if (character == 'a')
                                postKeyEvent<EvKeyPressA<CbKeyboard, TObjectTag>>();
                        else if (character == 'b')
                                postKeyEvent<EvKeyPressB<CbKeyboard, TObjectTag>>();
                        else if (character == 'c')
                                postKeyEvent<EvKeyPressC<CbKeyboard, TObjectTag>>();
                        else if (character == 'd')
                                postKeyEvent<EvKeyPressD<CbKeyboard, TObjectTag>>();
                        else if (character == 'e')
                                postKeyEvent<EvKeyPressE<CbKeyboard, TObjectTag>>();
                        else if (character == 'f')
                                postKeyEvent<EvKeyPressF<CbKeyboard, TObjectTag>>();
                        else if (character == 'g')
                                postKeyEvent<EvKeyPressG<CbKeyboard, TObjectTag>>();
                        else if (character == 'h')
                                postKeyEvent<EvKeyPressH<CbKeyboard, TObjectTag>>();
                        else if (character == 'y')
                                postKeyEvent<EvKeyPressI<CbKeyboard, TObjectTag>>();
                        else if (character == 'j')
                                postKeyEvent<EvKeyPressJ<CbKeyboard, TObjectTag>>();
                        else if (character == 'k')
                                postKeyEvent<EvKeyPressK<CbKeyboard, TObjectTag>>();
                        else if (character == 'l')
                                postKeyEvent<EvKeyPressL<CbKeyboard, TObjectTag>>();
                        else if (character == 'm')
                                postKeyEvent<EvKeyPressM<CbKeyboard, TObjectTag>>();
                        else if (character == 'n')
                                postKeyEvent<EvKeyPressN<CbKeyboard, TObjectTag>>();
                        else if (character == 'o')
                                postKeyEvent<EvKeyPressO<CbKeyboard, TObjectTag>>();
                        else if (character == 'p')
                                postKeyEvent<EvKeyPressP<CbKeyboard, TObjectTag>>();
                        else if (character == 'q')
                                postKeyEvent<EvKeyPressQ<CbKeyboard, TObjectTag>>();
                        else if (character == 'r')
                                postKeyEvent<EvKeyPressR<CbKeyboard, TObjectTag>>();
                        else if (character == 's')
                                postKeyEvent<EvKeyPressS<CbKeyboard, TObjectTag>>();
                        else if (character == 't')
                                postKeyEvent<EvKeyPressT<CbKeyboard, TObjectTag>>();
                        else if (character == 'u')
                                postKeyEvent<EvKeyPressU<CbKeyboard, TObjectTag>>();
                        else if (character == 'v')
                                postKeyEvent<EvKeyPressV<CbKeyboard, TObjectTag>>();
                        else if (character == 'w')
                                postKeyEvent<EvKeyPressW<CbKeyboard, TObjectTag>>();
                        else if (character == 'x')
                                postKeyEvent<EvKeyPressX<CbKeyboard, TObjectTag>>();
                        else if (character == 'y')
                                postKeyEvent<EvKeyPressY<CbKeyboard, TObjectTag>>();
                        else if (character == 'z')
                                postKeyEvent<EvKeyPressZ<CbKeyboard, TObjectTag>>();
                };
        }

        void OnKeyPress(char character)
        {
                postEventKeyPress(character);
        }

        template <typename TEv>
        void postKeyEvent()
        {
                ROS_WARN("CbKeyboard %ld ev: %s", (long)(void *)this, smacc::demangleSymbol(typeid(TEv).name()).c_str());
                auto event = new TEv();
                this->postEvent(event);
        }
};

} // namespace sm_three_some
