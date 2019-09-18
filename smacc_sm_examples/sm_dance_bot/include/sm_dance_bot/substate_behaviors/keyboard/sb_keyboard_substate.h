#pragma once

#include <smacc/smacc_substate_behavior.h>
#include <smacc/interface_components/smacc_topic_subscriber.h>

#include <boost/statechart/event.hpp>
#include <boost/asio/posix/stream_descriptor.hpp>

#include <boost/asio.hpp>
#include <iostream>
#include <thread>
#include <memory>
#include <std_msgs/UInt16.h>

using namespace boost::asio;

namespace smacc
{

//----------------- TIMER EVENT DEFINITION ----------------------------------------------
template <char keychar>
struct KeyPressEvent : sc::event<KeyPressEvent<keychar>>
{
};

//------------------  TIMER SUBSTATE ---------------------------------------------

template <typename TEventPoster>
void dispatchKeyEvent(char character, TEventPoster component);

class KeyboardClient : public smacc::SmaccTopicSubscriberClient<std_msgs::UInt16>
{
public:        
        virtual void initialize(std::string topicName, int queueSize) override
        {
                SmaccTopicSubscriberClient<std_msgs::UInt16>::initialize(topicName, queueSize);

                this->onMessageReceived.connect([this](auto msg) {
                        this->onKeyboardMessage(msg);
                });
        }

        void onKeyboardMessage(const std_msgs::UInt16 &unicode_keychar)
        {
                char character = (char)unicode_keychar.data;
                ROS_WARN("detected keyboard: %c", character);

                if (character == 'a')
                        postKeyEvent<'a'>();
                else if (character == 'b')
                        postKeyEvent<'b'>();
                else if (character == 'c')
                        postKeyEvent<'c'>();
                else if (character == 'd')
                        postKeyEvent<'d'>();
                else if (character == 'e')
                        postKeyEvent<'e'>();
                else if (character == 'f')
                        postKeyEvent<'f'>();
                else if (character == 'g')
                        postKeyEvent<'g'>();
                else if (character == 'h')
                        postKeyEvent<'h'>();
                else if (character == 'y')
                        postKeyEvent<'y'>();
                else if (character == 'j')
                        postKeyEvent<'j'>();
                else if (character == 'k')
                        postKeyEvent<'k'>();
                else if (character == 'l')
                        postKeyEvent<'l'>();
                else if (character == 'm')
                        postKeyEvent<'m'>();
                else if (character == 'n')
                        postKeyEvent<'n'>();
                else if (character == 'o')
                        postKeyEvent<'o'>();
                else if (character == 'p')
                        postKeyEvent<'p'>();
                else if (character == 'q')
                        postKeyEvent<'q'>();
                else if (character == 'r')
                        postKeyEvent<'r'>();
                else if (character == 's')
                        postKeyEvent<'s'>();
                else if (character == 't')
                        postKeyEvent<'t'>();
                else if (character == 'u')
                        postKeyEvent<'u'>();
                else if (character == 'v')
                        postKeyEvent<'v'>();
                else if (character == 'w')
                        postKeyEvent<'w'>();
                else if (character == 'x')
                        postKeyEvent<'x'>();
                else if (character == 'y')
                        postKeyEvent<'y'>();
                else if (character == 'z')
                        postKeyEvent<'z'>();
        }

public:
        template <char keychar>
        void postKeyEvent()
        {
                ROS_WARN("keypressed: %c", keychar);
                auto event = new KeyPressEvent<keychar>();
                this->postEvent(event);
        }
};

class SbKeyboard : public smacc::SmaccSubStateBehavior
{
public:
        KeyboardClient *keyboardClient_;

        void onEntry()
        {
                this->requiresComponent(keyboardClient_);
                keyboardClient_->initialize("/keyboard_unicode", 1);
        }

        bool onExit()
        {
        }
};

} // namespace smacc
