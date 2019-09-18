#pragma once

#include <smacc/smacc_substate_behavior.h>
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

class SbKeyboard : public smacc::SmaccSubStateBehavior
{
public:
  ros::NodeHandle nh_;
  ros::Subscriber sub_;

  int queueSize_;

  void onEntry()
  {
    queueSize_ = 10;
    ROS_INFO_STREAM("Subscribing to keyboard topic: keyboard_unicode");
    sub_ = nh_.subscribe("/keyboard_unicode", queueSize_, &SbKeyboard::keyboardMessage, this);
  }

  bool onExit()
  {
  }

  void keyboardMessage(const std_msgs::UInt16 &unicode_keychar)
  {
    char character =  (char)unicode_keychar.data;
    ROS_WARN("detected keyboard: %c", character);

    if( character == 'a')
            postKeyEvent<'a'>();
    else if( character == 'b')
            postKeyEvent<'b'>();
    else if( character == 'c')
            postKeyEvent<'c'>();
    else if( character == 'd')
            postKeyEvent<'d'>();
    else if( character == 'e')
            postKeyEvent<'e'>();
    else if( character == 'f')
            postKeyEvent<'f'>();
    else if( character == 'g')
            postKeyEvent<'g'>();
    else if( character == 'h')
            postKeyEvent<'h'>();
    else if( character == 'y')
            postKeyEvent<'y'>();
    else if( character == 'j')
            postKeyEvent<'j'>();
    else if( character == 'k')
            postKeyEvent<'k'>();
    else if( character == 'l')
            postKeyEvent<'l'>();
    else if( character == 'm')
            postKeyEvent<'m'>();
    else if( character == 'n')
            postKeyEvent<'n'>();
    else if( character == 'o')
            postKeyEvent<'o'>();
    else if( character == 'p')
            postKeyEvent<'p'>();
    else if( character == 'q')
            postKeyEvent<'q'>();
    else if( character == 'r')
            postKeyEvent<'r'>();
    else if( character == 's')
            postKeyEvent<'s'>();
    else if( character == 't')
            postKeyEvent<'t'>();
    else if( character == 'u')
            postKeyEvent<'u'>();
    else if( character == 'v')
            postKeyEvent<'v'>();
    else if( character == 'w')
            postKeyEvent<'w'>();
    else if( character == 'x')
            postKeyEvent<'x'>();
    else if( character == 'y')
            postKeyEvent<'y'>();
    else if( character == 'z')
            postKeyEvent<'z'>();
  }

  template <char keychar>
  void postKeyEvent()
  {
    ROS_WARN("keypressed: %c", keychar);
    auto event = new KeyPressEvent<keychar>();
    this->postEvent(event);
  }
};
} // namespace smacc
