#pragma once

#include <smacc/smacc.h>
#include <smacc/client_bases/smacc_subscriber_client.h>

#include <boost/asio/posix/stream_descriptor.hpp>
#include <boost/asio.hpp>
#include <iostream>
#include <thread>

#include <std_msgs/UInt16.h>

namespace keyboard_client
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

//------------------  KEYBOARD CLIENT ---------------------------------------------

class ClKeyboard : public smacc::client_bases::SmaccSubscriberClient<std_msgs::UInt16>
{
public:
        smacc::SmaccSignal<void(char keypress)> OnKeyPress_;

        template <typename T>
        void OnKeyPress(void (T::*callback)(char keypress), T *object)
        {
                //this->connectSignal(OnKeyPress_, callback, object);
                this->getStateMachine()->createSignalConnection(OnKeyPress_, callback, object);
        }

        ClKeyboard();

        virtual ~ClKeyboard();

        virtual void initialize() override;

        std::function<void(std_msgs::UInt16)> postEventKeyPress;

        template <typename TObjectTag, typename TDerived>
        void configureEventSourceTypes()
        {
                // call tothe base configuration to properly handling the message and initial message smacc events
                smacc::client_bases::SmaccSubscriberClient<std_msgs::UInt16>::configureEventSourceTypes<TObjectTag, TDerived>();

                postEventKeyPress = [=](auto unicode_keychar) {
                        char character = (char)unicode_keychar.data;
                        ROS_WARN("detected keyboard: %c", character);

                        if (character == 'a')
                                this->postKeyEvent<EvKeyPressA<ClKeyboard, TObjectTag>>();
                        else if (character == 'b')
                                this->postKeyEvent<EvKeyPressB<ClKeyboard, TObjectTag>>();
                        else if (character == 'c')
                                this->postKeyEvent<EvKeyPressC<ClKeyboard, TObjectTag>>();
                        else if (character == 'd')
                                this->postKeyEvent<EvKeyPressD<ClKeyboard, TObjectTag>>();
                        else if (character == 'e')
                                this->postKeyEvent<EvKeyPressE<ClKeyboard, TObjectTag>>();
                        else if (character == 'f')
                                this->postKeyEvent<EvKeyPressF<ClKeyboard, TObjectTag>>();
                        else if (character == 'g')
                                this->postKeyEvent<EvKeyPressG<ClKeyboard, TObjectTag>>();
                        else if (character == 'h')
                                this->postKeyEvent<EvKeyPressH<ClKeyboard, TObjectTag>>();
                        else if (character == 'y')
                                this->postKeyEvent<EvKeyPressI<ClKeyboard, TObjectTag>>();
                        else if (character == 'j')
                                this->postKeyEvent<EvKeyPressJ<ClKeyboard, TObjectTag>>();
                        else if (character == 'k')
                                this->postKeyEvent<EvKeyPressK<ClKeyboard, TObjectTag>>();
                        else if (character == 'l')
                                this->postKeyEvent<EvKeyPressL<ClKeyboard, TObjectTag>>();
                        else if (character == 'm')
                                this->postKeyEvent<EvKeyPressM<ClKeyboard, TObjectTag>>();
                        else if (character == 'n')
                                this->postKeyEvent<EvKeyPressN<ClKeyboard, TObjectTag>>();
                        else if (character == 'o')
                                this->postKeyEvent<EvKeyPressO<ClKeyboard, TObjectTag>>();
                        else if (character == 'p')
                                this->postKeyEvent<EvKeyPressP<ClKeyboard, TObjectTag>>();
                        else if (character == 'q')
                                this->postKeyEvent<EvKeyPressQ<ClKeyboard, TObjectTag>>();
                        else if (character == 'r')
                                this->postKeyEvent<EvKeyPressR<ClKeyboard, TObjectTag>>();
                        else if (character == 's')
                                this->postKeyEvent<EvKeyPressS<ClKeyboard, TObjectTag>>();
                        else if (character == 't')
                                this->postKeyEvent<EvKeyPressT<ClKeyboard, TObjectTag>>();
                        else if (character == 'u')
                                this->postKeyEvent<EvKeyPressU<ClKeyboard, TObjectTag>>();
                        else if (character == 'v')
                                this->postKeyEvent<EvKeyPressV<ClKeyboard, TObjectTag>>();
                        else if (character == 'w')
                                this->postKeyEvent<EvKeyPressW<ClKeyboard, TObjectTag>>();
                        else if (character == 'x')
                                this->postKeyEvent<EvKeyPressX<ClKeyboard, TObjectTag>>();
                        else if (character == 'y')
                                this->postKeyEvent<EvKeyPressY<ClKeyboard, TObjectTag>>();
                        else if (character == 'z')
                                this->postKeyEvent<EvKeyPressZ<ClKeyboard, TObjectTag>>();
                        OnKeyPress_(character); /*  */
                };
        }

        void onKeyboardMessage(const std_msgs::UInt16 &unicode_keychar);

        template <typename TEv>
        void postKeyEvent()
        {
                ROS_WARN("ClKeyboard ev: %s", smacc::demangleSymbol(typeid(TEv).name()).c_str());
                auto event = new TEv();
                this->postEvent(event);
        }

private:
        bool initialized_;
};
} // namespace cl_keyboard