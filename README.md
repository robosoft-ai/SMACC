# SMACC
SMACC is a c++ library that is used to create state machines in ROS in easy and systematic way. It is inspired in SMACH and it in Boost Statechart library.

## Cannonical SMACC applications
SMACC applications are canonically mobile robots (optionally with manipulators) with with tools that have to move around and interact with the environment.

## Features
 *  Integrated with ROS
 *  Static state machine consistence checking (inherited from Boost Statechart)
 *  Plugin based to control remote nodes based on ROS Action Client

## Development methodology
SMACC also defines a development methodology where State Machine nodes only contains the task-level logic, that is, the high level behavior of the robot system in some application.

SMACC applications have low level coupling with other software components of the robot system. SMACC code is recomended to interact with the rest of components the robot system via ROS Action Servers and **smacc action plugins**.

## Architecture

[Diagram]

## Repository Packages

This repository contains several ROS packages:

 * **smacc**: The core smacc library. It works as a template-based c++  header library.

 * **smacc_reelrbtx_plugin**: it implements a smacc-actionclien-plugin to handle the reel robotics reel device.

 * **smacc_navigation_plugin**: it implements a smacc-actionclient-plugin to handle the move_base node remotely.

 * **radial_motion_example**: shows a complete sample application developed with SMACC that can be reused as a canonical example of a mobile robot moving around and interacting with a custom onboard tool.

## Executing the Radial Motion Example
Requires: reelrbtx_control and reelrbtx_navigation packages and ridgeback packages

```
export RIDGEBACK_URDF_EXTRAS=$(rospack find reelrbtx_description)/urdf/reelrbtx.urdf.xacro

roslaunch radial_motion_example radial_motion.launch
```
## Tutorial

### Creating a simple state StateMachine with a single state

The state machine code:
```cpp
//-------------------- SIMPLE SMACC STATE MACHINE ----------

struct SimpleStateMachine
    : public SmaccStateMachineBase<SimpleStateMachine,ToolSimpleState> 
{
  SimpleStateMachine(my_context ctx, SignalDetector *signalDetector)
      : SmaccStateMachineBase<SimpleStateMachine,ToolSimpleState>(ctx, signalDetector) 
      {
      }
};
```
The state cpp code:

```cpp
// ------------------ SIMPLE STATE --------------
struct ToolSimpleState
    : SmaccState<ToolSimpleState,SimpleStateMachine> {
public:

  ToolSimpleState()
  {
    ROS_INFO("Entering into tool state");
  }

  ~ToolSimpleState()
  {
    ROS_INFO("Exiting from tool state");
  }
};
```

### Accessing to action client resources (AKA Action Client Plugins)

```cpp
struct ToolSubstate
    : statechart::simple_state<SimpleStateMachine> 
{
public:

  // This is the substate constructor. This code will be executed when the
  // workflow enters in this substate (that is according to statechart the moment when this object is created)
  ToolSubstate() 
  {
    ROS_INFO("Entering ToolSubstate");

    toolActionClient_ =
        context<SimpleStateMachine>().requiresActionClient<smacc::SmaccToolActionClient>("tool_action_server");

    smacc::SmaccToolActionClient::Goal goal;
    goal.command = smacc::SmaccToolActionClient::Goal::CMD_START;
    toolActionClient_->sendGoal(goal);
  }

  smacc::SmaccToolActionClient* toolActionClient_;
};
```


### Simple transitions between two states on action client callbacks
Describe sample here

### Transition between two states with custom code on action client callbacks
Describe sample here

### Creating global variables shared between states
Describe here

### Adding ros parameters to states
Describe sample here

### Creating orthogonal lines according to the SMACC methodology
Describe here