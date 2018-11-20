#include <smacc/smacc.h>

//--------------------------------------------------------------------------------------

using namespace smacc;

// states structs forward declarations
struct ToolSimpleState;

//--------------------------------------------------------------------------------------
struct SimpleStateMachine
    : public SmaccStateMachineBase<SimpleStateMachine,ToolSimpleState>
{
  SimpleStateMachine(my_context ctx, SignalDetector *signalDetector)
      : SmaccStateMachineBase<SimpleStateMachine,ToolSimpleState>(ctx, signalDetector)
      {
      }
};

//--------------------------------------------------------------------------------------
struct ToolSimpleState
    : SmaccState<ToolSimpleState, SimpleStateMachine>
{
public:

  // This is the substate constructor. This code will be executed when the
  // workflow enters in this substate (that is according to statechart the moment when this object is created)
  ToolSimpleState(my_context ctx)
    : SmaccState<ToolSimpleState, SimpleStateMachine>(ctx) // call the SmaccState base constructor
  {
    ROS_INFO("Entering ToolSimpleState");
  }
};

//--------------------------------------------------------------------------------------
int main(int argc, char **argv) {
  // initialize the ros node
  ros::init(argc, argv, "example1");
  ros::NodeHandle nh;

  smacc::run<SimpleStateMachine>();
}