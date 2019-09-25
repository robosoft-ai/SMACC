/*****************************************************************************************************************
 * ReelRobotix Inc. - Software License Agreement      Copyright (c) 2018
 * 	 Authors: Pablo Inigo Blasco, Brett Aldrich
 *
 ******************************************************************************************************************/
#pragma once
#include <smacc/common.h>
#include <smacc/smacc_state_machine_base.h>
#include <smacc/signal_detector.h>

namespace smacc
{
  // Main entry point for any SMACC state machine
  // It instanciates and starts the specified state machine type
  // it uses two threads: a new thread and the current one. 
  // The created thread is for the state machine process 
  // it locks the current thread to handle events of the state machine 
  template <typename StateMachineType>
  void run()
  {
    // create the asynchronous state machine scheduler
    SmaccScheduler scheduler1(true);

    // create the signalDetector component
    SignalDetector signalDetector(&scheduler1);

    // create the asynchronous state machine processor
    SmaccScheduler::processor_handle sm =
        scheduler1.create_processor<StateMachineType>(&signalDetector);
    
    // initialize the asynchronous state machine processor
    signalDetector.setProcessorHandle(sm);

    scheduler1.initiate_processor(sm);
    
    //create a thread for the asynchronous state machine processor execution
    boost::thread otherThread(boost::bind(&sc::fifo_scheduler<>::operator(), &scheduler1, 0));

    // use the  main thread for the signal detector component (waiting actionclient requests)
    signalDetector.pollingLoop();
  }
}