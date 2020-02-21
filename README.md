
## Travis CI: 

| ROS Distro  | Travis Build Status | Documentation |
| ------------- | ------------- | ------------- |
| Indigo  | <a src="https://travis-ci.org/reelrbtx/SMACC"><img src="https://travis-ci.com/reelrbtx/SMACC.svg?branch=indigo-devel"/> </a>|  <a href="https://reelrbtx.github.io/SMACC/indigo-devel/html/md_README.html">doxygen</a>|
| Kinetic  | <a src="https://travis-ci.org/reelrbtx/SMACC"><img src="https://travis-ci.com/reelrbtx/SMACC.svg?branch=kinetic-devel"/></a>  | <a href="https://reelrbtx.github.io/SMACC/kinetic-devel/html/md_README.html">doxygen</a>|
| Melodic  | <a src="https://travis-ci.org/reelrbtx/SMACC"><img src="https://travis-ci.com/reelrbtx/SMACC.svg?branch=melodic-devel"/></a>  | <a href="https://reelrbtx.github.io/SMACC/melodic-devel/html/md_README.html">doxygen</a>|
| Master  | <a src="https://travis-ci.org/reelrbtx/SMACC"><img src="https://travis-ci.com/reelrbtx/SMACC.svg?branch=master"/></a> | <a href="https://reelrbtx.github.io/SMACC_Documentation/master/html/namespaces.html">doxygen</a>|


## Docker Containers

[![Docker Automated build](https://img.shields.io/docker/automated/pabloinigoblasco/smacc.svg?maxAge=2592000)](https://hub.docker.com/r/pabloinigoblasco/smacc/) [![Docker Pulls](https://img.shields.io/docker/pulls/pabloinigoblasco/smacc.svg?maxAge=2592000)](https://hub.docker.com/r/pabloinigoblasco/smacc/) [![Docker Stars](https://img.shields.io/docker/stars/pabloinigoblasco/smacc.svg)](https://registry.hub.docker.com/pabloinigoblasco/smacc/)


# <img src="http://smacc.ninja/wp-content/uploads/2019/07/SMACC-Logo-Pixelate-4-copy.png" width="30" align="left"/> SMACC

SMACC is an event-driven, asynchronous, behavioral state machine library for real-time ROS (Robotic Operating System) applications written in C++, designed to allow programmers to build robot control applications for multicomponent robots, in an intuitive and systematic manner. 

SMACC is inspired by the [SMACH ROS package](http://wiki.ros.org/smach) and it is built on top of [Boost StateChart library](https://www.boost.org/doc/libs/1_53_0/libs/statechart/doc/index.html).

Probably the greatest strength of SMACC is that it offers out-of-the-box reference state machines, (found in the folder [sm_reference_library](https://github.com/reelrbtx/SMACC/tree/master/smacc_sm_reference_library))  that you can use, test, hack, and customize to quickly get your application up and running, while also knowing that the library supports advanced functionalities that are practically universal among actual working robots.

## Features
 *  ***Powered by ROS:*** SMACC has been developed specifically to work with ROS. It is a c++ ros package that can be imported from any end-user application package.
 *   ***Written in C++:*** Until now, ROS has lacked a library to develop task-level state machines in C++. Although libraries have been developed in scripting languages such as python, these are unsuitable for real-world industrial evironments where real-time requirements are demanded.
 *  ***Static State Machine Checking:*** SMACC inherits this from the Boost Statechart library which helps the developer to check the consistency of the state machine at compile time (instead of runtime).
 * ***Component based architecture:*** SMACC has built-in funcionality provided inside SMACC Components that can be dynamically imported at runtime and stored in the local machine. The states only access those components they are concerned with. This enables the SMACC based application to extend or improve the runtime behavior of the system.

## SMACC applications
From it's inception, SMACC was written to support the programming of multi-component, complex robots. If your project involves small, solar-powered insect robots, that simply navigate towards a light source, then SMACC might not be the right choice for you. But if you are trying to program a robot with a mobile base, a robotic arm, a gripper, two lidar sensors, a gps transceiver and an imu, then you've come to the right place. 

## ROS Integration

* ***Intensive use of ROS Action***. SMACC translates Action server events (Result callbacks, Feedback callbacks, etc.) into statechart events. To learn more about this, check the sections Shared Resources and SMACC Architecture.
* ***Powerful access to ROS Parameters***. Each SMACC state automatically creates a ros::NodeHandle automatically named according to the SMACC state hierarchy (see more in section Usage Examples - Ros parameters)
* ***ROS Navigation built-in funcionality***. SMACC extends the ROS navigation stack in a high level way. It provides specialized navigation planners (for the ROS Navigation Stack) that navigate only using pure spinning motions and straight motions. Implements some mechanism to perform motions recording the path and undoing them later.
These can be very useful in some industrial applications where the knowledge or certainty on the environment is higher (ros planners are focused on cluttered and dynamic environments).

## Future Work
 * undoing paths chunks by state (store the different chunks of the path according to its state in a stack)
 * code generation based on uml diagrams
 * improving backwards planners for non linear paths
 
## Development methodology
SMACC also defines a development methodology where State Machine nodes only contain the task-level logic, that is, the high level behavior of the robot system in some specific application.

SMACC applications have low level coupling with other software components of the robot system. SMACC code is recomended to interact with the rest of components the robot system via ROS Action Servers and **Smacc Action Plugins**.

The proposed methdology split the states into 2 or more statechart orthogonal lines that comunicate to each other via events. The orthogonal line 0 is tipically for the mobile robot navigation. The second orthogonal line and ahead are used for tools (manipulators, grippers or other custom tools).

<p align="center">
<img src="https://github.com/reelrbtx/SMACC/blob/master/documentation/SMACC-Containers-3.jpg"  width="450" align="center"/>
</p>

## Internal Architecture
SMACC State Machines are boost::statechart AsynchronousStateMachines that can work in a multi-threaded application. In SMACC State Machines are two main components that work concurrently in two different threads:

* ***Signal Detector***. It is able to handle the action client components communication with action servers and translate them to statechart events
* ***State Machine***. It is the end-user code of the state machine itself.

<p align="center">
<img src="http://smacc.ninja/wp-content/uploads/2018/09/SMACC-Node-Map-2-2-1.jpg"  width="450" align="center"/>
</p>

# Tutorial
SMACC states inherits from boost::statechart:State so that you can learn the full potential of SMACC states also diving in the statechart documentation. However, the following examples briefly show how you create define SMACC states and how you would usually use them.

## Getting Started
The easiest way to get started is by selecting one of the state machines in our reference library, and then hacking it to meet your needs.

Each state machine in the reference library comes with it's own README.md file, which contains the appropriate operating instructions, so that all you have to do is simply copy & paste some commands into your terminal.

## Anatomy of a simple SMACC State

For the previous state machine, this would be the initial SMACC State. It also follows the Curiously recurrent template pattern. However, for Smacc states, the second template parameters is the so called "Context", for this simple case, the context is the StateMachine type itself. However, that could also be other State (in a nexted-substate case) or an orthogonal line.

```cpp
struct ToolSimpleState
    : SmaccState<ToolSimpleState, SimpleStateMachine>
{
public:

  using SmaccState::SmaccState;
  void onEntry()
  {
    ROS_INFO("Entering ToolSimpleState");
  }
};

int main(int argc, char **argv) {
  // initialize the ros node
  ros::init(argc, argv, "example1");
  ros::NodeHandle nh;

  smacc::run<SimpleStateMachine>();
}
```
According to the UML statchart standard, things happens essencially when the system enters in the state, when the system exits the state and when some event is triggered. The two first ones are shown in this example. The c++ Constructor code is the place you have to write your "entry code", the destructor is the place you have to write your "exit code". The constructor parameter (my_context) is a reference to the context object (in this case the state machine). This kind of constructor may be verebosy, but is required to implement the rest of SMACC tasks and always follows the same pattern.


## Simple State Transition on Action Result Event

According to the UML state machines standard, transitions between states happen on events. In SMACC events can be implemented by the user or happen
when Action Results callbacks and Action Feedback callbacks happen. In the following example we extend the previous example to transit to another state 'ExecuteToolState' when the move_base
action sever returns a Result.


<p align="center">
<img src="https://raw.githubusercontent.com/brettpac/SMACC/master/documentation/action_result_transition.png" width="450"/>
</p>

The following would be the code to implement the diagram shown above.

```cpp
struct Navigate : SmaccState<Navigate, SimpleStateMachine>
{
public:

  // With this line we specify that we are going to react to any EvActionResult event
  // generated by SMACC when the action server provides a response to our request
  typedef mpl::list<sc::transition<EvActionResult<SmaccMoveBaseActionClient::Result>, ExecuteToolState>> reactions;

  using SmaccState::SmaccState;
  void onEntry()
  {
   [...]
  }
};

struct ExecuteToolState : SmaccState<ExecuteToolState, SimpleStateMachine>
{
    using SmaccState::SmaccState;
    void onEntry()
    {
    }
};
```

