#pragma once

#include <keyboard_client/cl_keyboard.h>
#include <smacc/smacc_client_behavior.h>

#include <std_msgs/UInt16.h>

namespace cl_keyboard
{
class CbDefaultKeyboardBehavior : public smacc::SmaccClientBehavior
{
public:
    ClKeyboard *ClKeyboard_;
    std::function<void(char)> postEventKeyPress;

    void onEntry()
    {
        this->requiresClient(ClKeyboard_);
        this->ClKeyboard_->OnKeyPress(&CbDefaultKeyboardBehavior::OnKeyPress, this);
    }

    template <typename TObjectTag, typename TDerived>
    void configureEventSourceTypes()
    {
        postEventKeyPress = [=](char character) {
            if (character == 'a')
                postKeyEvent<EvKeyPressA<CbDefaultKeyboardBehavior, TObjectTag>>();
            else if (character == 'b')
                postKeyEvent<EvKeyPressB<CbDefaultKeyboardBehavior, TObjectTag>>();
            else if (character == 'c')
                postKeyEvent<EvKeyPressC<CbDefaultKeyboardBehavior, TObjectTag>>();
            else if (character == 'd')
                postKeyEvent<EvKeyPressD<CbDefaultKeyboardBehavior, TObjectTag>>();
            else if (character == 'e')
                postKeyEvent<EvKeyPressE<CbDefaultKeyboardBehavior, TObjectTag>>();
            else if (character == 'f')
                postKeyEvent<EvKeyPressF<CbDefaultKeyboardBehavior, TObjectTag>>();
            else if (character == 'g')
                postKeyEvent<EvKeyPressG<CbDefaultKeyboardBehavior, TObjectTag>>();
            else if (character == 'h')
                postKeyEvent<EvKeyPressH<CbDefaultKeyboardBehavior, TObjectTag>>();
            else if (character == 'i')
                postKeyEvent<EvKeyPressI<CbDefaultKeyboardBehavior, TObjectTag>>();
            else if (character == 'j')
                postKeyEvent<EvKeyPressJ<CbDefaultKeyboardBehavior, TObjectTag>>();
            else if (character == 'k')
                postKeyEvent<EvKeyPressK<CbDefaultKeyboardBehavior, TObjectTag>>();
            else if (character == 'l')
                postKeyEvent<EvKeyPressL<CbDefaultKeyboardBehavior, TObjectTag>>();
            else if (character == 'm')
                postKeyEvent<EvKeyPressM<CbDefaultKeyboardBehavior, TObjectTag>>();
            else if (character == 'n')
                postKeyEvent<EvKeyPressN<CbDefaultKeyboardBehavior, TObjectTag>>();
            else if (character == 'o')
                postKeyEvent<EvKeyPressO<CbDefaultKeyboardBehavior, TObjectTag>>();
            else if (character == 'p')
                postKeyEvent<EvKeyPressP<CbDefaultKeyboardBehavior, TObjectTag>>();
            else if (character == 'q')
                postKeyEvent<EvKeyPressQ<CbDefaultKeyboardBehavior, TObjectTag>>();
            else if (character == 'r')
                postKeyEvent<EvKeyPressR<CbDefaultKeyboardBehavior, TObjectTag>>();
            else if (character == 's')
                postKeyEvent<EvKeyPressS<CbDefaultKeyboardBehavior, TObjectTag>>();
            else if (character == 't')
                postKeyEvent<EvKeyPressT<CbDefaultKeyboardBehavior, TObjectTag>>();
            else if (character == 'u')
                postKeyEvent<EvKeyPressU<CbDefaultKeyboardBehavior, TObjectTag>>();
            else if (character == 'v')
                postKeyEvent<EvKeyPressV<CbDefaultKeyboardBehavior, TObjectTag>>();
            else if (character == 'w')
                postKeyEvent<EvKeyPressW<CbDefaultKeyboardBehavior, TObjectTag>>();
            else if (character == 'x')
                postKeyEvent<EvKeyPressX<CbDefaultKeyboardBehavior, TObjectTag>>();
            else if (character == 'y')
                postKeyEvent<EvKeyPressY<CbDefaultKeyboardBehavior, TObjectTag>>();
            else if (character == 'z')
                postKeyEvent<EvKeyPressZ<CbDefaultKeyboardBehavior, TObjectTag>>();
        };
    }

    void OnKeyPress(char character)
    {
        postEventKeyPress(character);
    }

    template <typename TEv>
    void postKeyEvent()
    {
        ROS_WARN("CbDefaultKeyboardBehavior %ld ev: %s", (long)(void *)this, smacc::demangleSymbol(typeid(TEv).name()).c_str());
        auto event = new TEv();
        this->postEvent(event);
    }
};
} // namespace cl_keyboard