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

namespace dance_bot
{

//----------------- TIMER sc::event DEFINITION ----------------------------------------------
template <typename TSource>
struct EvKeyPressA : sc::event<EvKeyPressA<TSource>>
{
};

template <typename TSource>
struct EvKeyPressB : sc::event<EvKeyPressB<TSource>>
{
};

template <typename TSource>
struct EvKeyPressC : sc::event<EvKeyPressC<TSource>>
{
};

template <typename TSource>
struct EvKeyPressD : sc::event<EvKeyPressD<TSource>>
{
};

template <typename TSource>
struct EvKeyPressE : sc::event<EvKeyPressE<TSource>>
{
};

template <typename TSource>
struct EvKeyPressF : sc::event<EvKeyPressF<TSource>>
{
};

template <typename TSource>
struct EvKeyPressG : sc::event<EvKeyPressG<TSource>>
{
};

template <typename TSource>
struct EvKeyPressH : sc::event<EvKeyPressH<TSource>>
{
};

template <typename TSource>
struct EvKeyPressI : sc::event<EvKeyPressI<TSource>>
{
};

template <typename TSource>
struct EvKeyPressJ : sc::event<EvKeyPressJ<TSource>>
{
};

template <typename TSource>
struct EvKeyPressK : sc::event<EvKeyPressK<TSource>>
{
};

template <typename TSource>
struct EvKeyPressL : sc::event<EvKeyPressL<TSource>>
{
};

template <typename TSource>
struct EvKeyPressM : sc::event<EvKeyPressM<TSource>>
{
};

template <typename TSource>
struct EvKeyPressN : sc::event<EvKeyPressN<TSource>>
{
};

template <typename TSource>
struct EvKeyPressO : sc::event<EvKeyPressO<TSource>>
{
};

template <typename TSource>
struct EvKeyPressP : sc::event<EvKeyPressP<TSource>>
{
};

template <typename TSource>
struct EvKeyPressQ : sc::event<EvKeyPressQ<TSource>>
{
};

template <typename TSource>
struct EvKeyPressR : sc::event<EvKeyPressR<TSource>>
{
};

template <typename TSource>
struct EvKeyPressS : sc::event<EvKeyPressS<TSource>>
{
};

template <typename TSource>
struct EvKeyPressT : sc::event<EvKeyPressT<TSource>>
{
};

template <typename TSource>
struct EvKeyPressU : sc::event<EvKeyPressU<TSource>>
{
};

template <typename TSource>
struct EvKeyPressV : sc::event<EvKeyPressV<TSource>>
{
};

template <typename TSource>
struct EvKeyPressW : sc::event<EvKeyPressW<TSource>>
{
};

template <typename TSource>
struct EvKeyPressX : sc::event<EvKeyPressX<TSource>>
{
};

template <typename TSource>
struct EvKeyPressY : sc::event<EvKeyPressY<TSource>>
{
};

template <typename TSource>
struct EvKeyPressZ : sc::event<EvKeyPressZ<TSource>>
{
};

//------------------  TIMER SUBSTATE ---------------------------------------------

class KeyboardClient : public smacc::SmaccTopicSubscriberClient<KeyboardClient, std_msgs::UInt16>
{
public:
        boost::signals2::signal<void(char keypress)> OnKeyPress;
        boost::signals2::scoped_connection c_;

        KeyboardClient()
        {
                initialized_ = false;
        }

        virtual ~KeyboardClient()
        {
        }

        virtual void initialize() override
        {
                SmaccTopicSubscriberClient<KeyboardClient, std_msgs::UInt16>::initialize();

                if (!this->initialized_)
                {
                        c_ = this->onMessageReceived.connect([this](auto msg) {
                                this->onKeyboardMessage(msg);

                                this->initialized_ = true;
                        });
                }
        }

        void onKeyboardMessage(const std_msgs::UInt16 &unicode_keychar)
        {
                char character = (char)unicode_keychar.data;
                ROS_WARN("detected keyboard: %c", character);

                if (character == 'a')
                        postKeyEvent<EvKeyPressA<KeyboardClient>>();
                else if (character == 'b')
                        postKeyEvent<EvKeyPressB<KeyboardClient>>();
                else if (character == 'c')
                        postKeyEvent<EvKeyPressC<KeyboardClient>>();
                else if (character == 'd')
                        postKeyEvent<EvKeyPressD<KeyboardClient>>();
                else if (character == 'e')
                        postKeyEvent<EvKeyPressE<KeyboardClient>>();
                else if (character == 'f')
                        postKeyEvent<EvKeyPressF<KeyboardClient>>();
                else if (character == 'g')
                        postKeyEvent<EvKeyPressG<KeyboardClient>>();
                else if (character == 'h')
                        postKeyEvent<EvKeyPressH<KeyboardClient>>();
                else if (character == 'y')
                        postKeyEvent<EvKeyPressI<KeyboardClient>>();
                else if (character == 'j')
                        postKeyEvent<EvKeyPressJ<KeyboardClient>>();
                else if (character == 'k')
                        postKeyEvent<EvKeyPressK<KeyboardClient>>();
                else if (character == 'l')
                        postKeyEvent<EvKeyPressL<KeyboardClient>>();
                else if (character == 'm')
                        postKeyEvent<EvKeyPressM<KeyboardClient>>();
                else if (character == 'n')
                        postKeyEvent<EvKeyPressN<KeyboardClient>>();
                else if (character == 'o')
                        postKeyEvent<EvKeyPressO<KeyboardClient>>();
                else if (character == 'p')
                        postKeyEvent<EvKeyPressP<KeyboardClient>>();
                else if (character == 'q')
                        postKeyEvent<EvKeyPressQ<KeyboardClient>>();
                else if (character == 'r')
                        postKeyEvent<EvKeyPressR<KeyboardClient>>();
                else if (character == 's')
                        postKeyEvent<EvKeyPressS<KeyboardClient>>();
                else if (character == 't')
                        postKeyEvent<EvKeyPressT<KeyboardClient>>();
                else if (character == 'u')
                        postKeyEvent<EvKeyPressU<KeyboardClient>>();
                else if (character == 'v')
                        postKeyEvent<EvKeyPressV<KeyboardClient>>();
                else if (character == 'w')
                        postKeyEvent<EvKeyPressW<KeyboardClient>>();
                else if (character == 'x')
                        postKeyEvent<EvKeyPressX<KeyboardClient>>();
                else if (character == 'y')
                        postKeyEvent<EvKeyPressY<KeyboardClient>>();
                else if (character == 'z')
                        postKeyEvent<EvKeyPressZ<KeyboardClient>>();

                OnKeyPress(character);
        }

        template <typename TEv>
        void postKeyEvent()
        {
                ROS_WARN("KeyboardClient ev: %s", demangleSymbol(typeid(TEv).name()).c_str());
                auto event = new TEv();
                this->postEvent(event);
        }

private:
        bool initialized_;
};
} // namespace smacc