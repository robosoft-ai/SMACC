#pragma once

#include <smacc/smacc.h>
#include <smacc/client_bases/smacc_subscriber_client.h>

#include <boost/asio/posix/stream_descriptor.hpp>
#include <boost/asio.hpp>
#include <iostream>
#include <thread>

#include <std_msgs/UInt16.h>

namespace sm_three_some
{

//----------------- TIMER sc::event DEFINITION ----------------------------------------------
template <typename TSource, typename TObjectTag>
struct EvKeyPressA : sc::event<EvKeyPressA<TSource, TObjectTag>>
{
};

template <typename TSource, typename TObjectTag>
struct EvKeyPressB : sc::event<EvKeyPressB<TSource, TObjectTag>>
{
};

template <typename TSource, typename TObjectTag>
struct EvKeyPressC : sc::event<EvKeyPressC<TSource, TObjectTag>>
{
};

template <typename TSource, typename TObjectTag>
struct EvKeyPressD : sc::event<EvKeyPressD<TSource, TObjectTag>>
{
};

template <typename TSource, typename TObjectTag>
struct EvKeyPressE : sc::event<EvKeyPressE<TSource, TObjectTag>>
{
};

template <typename TSource, typename TObjectTag>
struct EvKeyPressF : sc::event<EvKeyPressF<TSource, TObjectTag>>
{
};

template <typename TSource, typename TObjectTag>
struct EvKeyPressG : sc::event<EvKeyPressG<TSource, TObjectTag>>
{
};

template <typename TSource, typename TObjectTag>
struct EvKeyPressH : sc::event<EvKeyPressH<TSource, TObjectTag>>
{
};

template <typename TSource, typename TObjectTag>
struct EvKeyPressI : sc::event<EvKeyPressI<TSource, TObjectTag>>
{
};

template <typename TSource, typename TObjectTag>
struct EvKeyPressJ : sc::event<EvKeyPressJ<TSource, TObjectTag>>
{
};

template <typename TSource, typename TObjectTag>
struct EvKeyPressK : sc::event<EvKeyPressK<TSource, TObjectTag>>
{
};

template <typename TSource, typename TObjectTag>
struct EvKeyPressL : sc::event<EvKeyPressL<TSource, TObjectTag>>
{
};

template <typename TSource, typename TObjectTag>
struct EvKeyPressM : sc::event<EvKeyPressM<TSource, TObjectTag>>
{
};

template <typename TSource, typename TObjectTag>
struct EvKeyPressN : sc::event<EvKeyPressN<TSource, TObjectTag>>
{
};

template <typename TSource, typename TObjectTag>
struct EvKeyPressO : sc::event<EvKeyPressO<TSource, TObjectTag>>
{
};

template <typename TSource, typename TObjectTag>
struct EvKeyPressP : sc::event<EvKeyPressP<TSource, TObjectTag>>
{
};

template <typename TSource, typename TObjectTag>
struct EvKeyPressQ : sc::event<EvKeyPressQ<TSource, TObjectTag>>
{
};

template <typename TSource, typename TObjectTag>
struct EvKeyPressR : sc::event<EvKeyPressR<TSource, TObjectTag>>
{
};

template <typename TSource, typename TObjectTag>
struct EvKeyPressS : sc::event<EvKeyPressS<TSource, TObjectTag>>
{
};

template <typename TSource, typename TObjectTag>
struct EvKeyPressT : sc::event<EvKeyPressT<TSource, TObjectTag>>
{
};

template <typename TSource, typename TObjectTag>
struct EvKeyPressU : sc::event<EvKeyPressU<TSource, TObjectTag>>
{
};

template <typename TSource, typename TObjectTag>
struct EvKeyPressV : sc::event<EvKeyPressV<TSource, TObjectTag>>
{
};

template <typename TSource, typename TObjectTag>
struct EvKeyPressW : sc::event<EvKeyPressW<TSource, TObjectTag>>
{
};

template <typename TSource, typename TObjectTag>
struct EvKeyPressX : sc::event<EvKeyPressX<TSource, TObjectTag>>
{
};

template <typename TSource, typename TObjectTag>
struct EvKeyPressY : sc::event<EvKeyPressY<TSource, TObjectTag>>
{
};

template <typename TSource, typename TObjectTag>
struct EvKeyPressZ : sc::event<EvKeyPressZ<TSource, TObjectTag>>
{
};

//------------------  TIMER SUBSTATE ---------------------------------------------

class KeyboardClient : public smacc::SmaccSubscriberClient<std_msgs::UInt16>
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
                SmaccSubscriberClient<std_msgs::UInt16>::initialize();

                if (!this->initialized_)
                {
                        c_ = this->onMessageReceived.connect([this](auto msg) {
                                this->onKeyboardMessage(msg);

                                this->initialized_ = true;
                        });
                }
        }

        std::function<void(std_msgs::UInt16)> postEventKeyPress;

        template <typename TDerived, typename TObjectTag>
        void assignToOrthogonal()
        {
                postEventKeyPress = [=](auto unicode_keychar) {
                        char character = (char)unicode_keychar.data;
                        ROS_WARN("detected keyboard: %c", character);

                        if (character == 'a')
                                postKeyEvent<EvKeyPressA<KeyboardClient, TObjectTag>>();
                        else if (character == 'b')
                                postKeyEvent<EvKeyPressB<KeyboardClient, TObjectTag>>();
                        else if (character == 'c')
                                postKeyEvent<EvKeyPressC<KeyboardClient, TObjectTag>>();
                        else if (character == 'd')
                                postKeyEvent<EvKeyPressD<KeyboardClient, TObjectTag>>();
                        else if (character == 'e')
                                postKeyEvent<EvKeyPressE<KeyboardClient, TObjectTag>>();
                        else if (character == 'f')
                                postKeyEvent<EvKeyPressF<KeyboardClient, TObjectTag>>();
                        else if (character == 'g')
                                postKeyEvent<EvKeyPressG<KeyboardClient, TObjectTag>>();
                        else if (character == 'h')
                                postKeyEvent<EvKeyPressH<KeyboardClient, TObjectTag>>();
                        else if (character == 'y')
                                postKeyEvent<EvKeyPressI<KeyboardClient, TObjectTag>>();
                        else if (character == 'j')
                                postKeyEvent<EvKeyPressJ<KeyboardClient, TObjectTag>>();
                        else if (character == 'k')
                                postKeyEvent<EvKeyPressK<KeyboardClient, TObjectTag>>();
                        else if (character == 'l')
                                postKeyEvent<EvKeyPressL<KeyboardClient, TObjectTag>>();
                        else if (character == 'm')
                                postKeyEvent<EvKeyPressM<KeyboardClient, TObjectTag>>();
                        else if (character == 'n')
                                postKeyEvent<EvKeyPressN<KeyboardClient, TObjectTag>>();
                        else if (character == 'o')
                                postKeyEvent<EvKeyPressO<KeyboardClient, TObjectTag>>();
                        else if (character == 'p')
                                postKeyEvent<EvKeyPressP<KeyboardClient, TObjectTag>>();
                        else if (character == 'q')
                                postKeyEvent<EvKeyPressQ<KeyboardClient, TObjectTag>>();
                        else if (character == 'r')
                                postKeyEvent<EvKeyPressR<KeyboardClient, TObjectTag>>();
                        else if (character == 's')
                                postKeyEvent<EvKeyPressS<KeyboardClient, TObjectTag>>();
                        else if (character == 't')
                                postKeyEvent<EvKeyPressT<KeyboardClient, TObjectTag>>();
                        else if (character == 'u')
                                postKeyEvent<EvKeyPressU<KeyboardClient, TObjectTag>>();
                        else if (character == 'v')
                                postKeyEvent<EvKeyPressV<KeyboardClient, TObjectTag>>();
                        else if (character == 'w')
                                postKeyEvent<EvKeyPressW<KeyboardClient, TObjectTag>>();
                        else if (character == 'x')
                                postKeyEvent<EvKeyPressX<KeyboardClient, TObjectTag>>();
                        else if (character == 'y')
                                postKeyEvent<EvKeyPressY<KeyboardClient, TObjectTag>>();
                        else if (character == 'z')
                                postKeyEvent<EvKeyPressZ<KeyboardClient, TObjectTag>>();
                        OnKeyPress(character); /*  */
                };
        }

        void onKeyboardMessage(const std_msgs::UInt16 &unicode_keychar)
        {
                postEventKeyPress(unicode_keychar);
        }

        template <typename TEv>
        void postKeyEvent()
        {
                ROS_WARN("KeyboardClient ev: %s", smacc::demangleSymbol(typeid(TEv).name()).c_str());
                auto event = new TEv();
                this->postEvent(event);
        }

private:
        bool initialized_;
};
} // namespace sm_three_some