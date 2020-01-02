#include <keyboard_client/cl_keyboard.h>

namespace keyboard_client
{
ClKeyboard::ClKeyboard()
{
    initialized_ = false;
    topicName = "/keyboard_unicode";
}

ClKeyboard::~ClKeyboard()
{
}

void ClKeyboard::initialize()
{

    SmaccSubscriberClient<std_msgs::UInt16>::initialize();

    if (!this->initialized_)
    {
        c_ = this->onMessageReceived.connect([this](auto msg) {
            this->onKeyboardMessage(msg);

            this->initialized_ = true;
        });
    }
}

void ClKeyboard::onKeyboardMessage(const std_msgs::UInt16 &unicode_keychar)
{
    postEventKeyPress(unicode_keychar);
}
} // namespace keyboard_client