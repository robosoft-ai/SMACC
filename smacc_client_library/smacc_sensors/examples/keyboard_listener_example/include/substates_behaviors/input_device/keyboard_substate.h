#pragma once

#include <smacc/smacc_substate_behavior.h>
#include <boost/statechart/event.hpp>
#include <boost/asio/posix/stream_descriptor.hpp>


#include <boost/asio.hpp>
#include <iostream>

using namespace boost::asio;


namespace smacc
{

//----------------- TIMER EVENT DEFINITION ----------------------------------------------
template <char keychar>
struct KeyPressEvent: sc::event< KeyPressEvent<keychar>>
{
 
};

//------------------  TIMER SUBSTATE ---------------------------------------------

class Keyboard : public smacc::SmaccSubStateBehavior
{
  
 public:
  void onEntry()
  {
  }

  void onExit()
  {
  }

  void keyboardListenerLoop()
  {
    io_service ioservice;        
    boost::asio::posix::stream_descriptor stream(ioservice, STDIN_FILENO);

    char buf[1] = {};

    std::function<void(boost::system::error_code, size_t)> read_handler;

    read_handler = [&](boost::system::error_code ec, size_t len) {   
            if (ec) {
                std::cerr << "exit with " << ec.message() << std::endl;
            } else {
                
                if (len == 1 ) {

                  if(buf[0]=='a') postKeyEvent<'a'>();
                  else if(buf[0]=='a') postKeyEvent<'a'>();
                  else if(buf[0]=='b') postKeyEvent<'b'>();
                  else if(buf[0]=='c') postKeyEvent<'c'>();
                  else if(buf[0]=='d') postKeyEvent<'d'>();
                  else if(buf[0]=='e') postKeyEvent<'e'>();
                  else if(buf[0]=='f') postKeyEvent<'f'>();
                  else if(buf[0]=='g') postKeyEvent<'g'>();
                  else if(buf[0]=='h') postKeyEvent<'h'>();
                  else if(buf[0]=='y') postKeyEvent<'y'>();
                  else if(buf[0]=='j') postKeyEvent<'j'>();
                  else if(buf[0]=='k') postKeyEvent<'k'>();
                  else if(buf[0]=='l') postKeyEvent<'l'>();
                  else if(buf[0]=='m') postKeyEvent<'m'>();
                  else if(buf[0]=='n') postKeyEvent<'n'>();
                  else if(buf[0]=='o') postKeyEvent<'o'>();
                  else if(buf[0]=='p') postKeyEvent<'p'>();
                  else if(buf[0]=='q') postKeyEvent<'q'>();
                  else if(buf[0]=='r') postKeyEvent<'r'>();
                  else if(buf[0]=='s') postKeyEvent<'s'>();
                  else if(buf[0]=='t') postKeyEvent<'t'>();
                  else if(buf[0]=='u') postKeyEvent<'u'>();
                  else if(buf[0]=='v') postKeyEvent<'v'>();
                  else if(buf[0]=='w') postKeyEvent<'w'>();
                  else if(buf[0]=='x') postKeyEvent<'x'>();
                  else if(buf[0]=='y') postKeyEvent<'y'>();
                  else if(buf[0]=='z') postKeyEvent<'z'>();
               
                }
                async_read(stream, buffer(buf), read_handler);
            }
        };

    async_read(stream, buffer(buf), read_handler);

    ioservice.run();    
  }

  template <char keychar>
  void postKeyEvent()
  {
    auto event= new KeyPressEvent<keychar>();
    this->postEvent(event);

  }      
};
}
