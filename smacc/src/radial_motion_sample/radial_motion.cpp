#include "radial_motion.h"
#include "smacc/signal_detector.h"
#include "states/navigate_to_radial_start.h"
#include "states/rotate_ten_degrees.h"
#include "states/return_to_radial_start.h"
#include "states/navigate_to_end_point.h"
#include <boost/thread.hpp>

//------------------------------------------------------------------------------

int main(int argc, char** argv)
{
    ros::init(argc,argv, "radial_test_state_machine");
    ros::NodeHandle nh;

    ROS_INFO("Hello World!");

    auto shared_request_signal = std::make_shared<boost::signals2::signal<void (std::string actionClientRequestInfo) >>();

    SmaccScheduler scheduler1( true );
    
    SignalDetector signalDetector(&scheduler1);
    SmaccScheduler::processor_handle radialMotionStateMachine = scheduler1.create_processor< RadialMotionStateMachine >(&signalDetector);
    signalDetector.setProcessorHandle(radialMotionStateMachine);
    scheduler1.initiate_processor( radialMotionStateMachine);

    boost::thread otherThread( boost::bind(
    &sc::fifo_scheduler<>::operator(), &scheduler1, 0 ) );

    signalDetector.simulateResponses();

    ros::Duration(10).sleep();
}