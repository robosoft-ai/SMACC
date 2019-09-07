
struct RadialMotionWaypointsStateMachine
    : public smacc::SmaccStateMachineBase<RadialMotionWaypointsStateMachine,RadialMotion1::RadialMotion1> 
{
    RadialMotionWaypointsStateMachine(my_context ctx, smacc::SignalDetector *signalDetector)
      : SmaccStateMachineBase<RadialMotionWaypointsStateMachine,RadialMotion1::RadialMotion1>(ctx, signalDetector) 
      {
      }      

};