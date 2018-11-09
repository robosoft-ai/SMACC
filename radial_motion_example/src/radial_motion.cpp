#include <radial_motion.h>
#include <smacc/signal_detector.h>
#include <states/navigate_to_end_point.h>
#include <states/navigate_to_radial_start.h>
#include <states/return_to_radial_start.h>
#include <states/rotate_degrees.h>
#include <boost/thread.hpp>

//------------------------------------------------------------------------------

int main(int argc, char **argv) {
  // initialize the ros node
  ros::init(argc, argv, "radial_test_state_machine");
  ros::NodeHandle nh;

  ROS_INFO("Hello World!");

  // create the asynchronous state machine scheduler
  SmaccScheduler scheduler1(true);

  // create the signalDetector component
  SignalDetector signalDetector(&scheduler1);

  // create the asynchronous state machine processor
  SmaccScheduler::processor_handle radialMotionStateMachine =
      scheduler1.create_processor<RadialMotionStateMachine>(&signalDetector);
  
  // initialize the asynchronous state machine processor
  signalDetector.setProcessorHandle(radialMotionStateMachine);
  scheduler1.initiate_processor(radialMotionStateMachine);

  //create a thread for the asynchronous state machine processor execution
  boost::thread otherThread(
      boost::bind(&sc::fifo_scheduler<>::operator(), &scheduler1, 0));

  // use the  main thread for the signal detector component (waiting actionclient requests)
  signalDetector.pollingLoop();

  ros::Duration(10).sleep();
}