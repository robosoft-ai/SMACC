
## Travis CI: 

| ROS Distro  | Travis Build Status | Documentation |
| ------------- | ------------- | ------------- |
| Indigo  | <a src="https://travis-ci.org/reelrbtx/SMACC"><img src="https://travis-ci.com/reelrbtx/SMACC.svg?branch=indigo-devel"/> </a>|  <a href="https://reelrbtx.github.io/SMACC/indigo-devel/html/md_README.html">doxygen</a>|
| Kinetic  | <a src="https://travis-ci.org/reelrbtx/SMACC"><img src="https://travis-ci.com/reelrbtx/SMACC.svg?branch=kinetic-devel"/></a>  | <a href="https://reelrbtx.github.io/SMACC/kinetic-devel/html/md_README.html">doxygen</a>|
| Melodic  | <a src="https://travis-ci.org/reelrbtx/SMACC"><img src="https://travis-ci.com/reelrbtx/SMACC.svg?branch=melodic-devel"/></a>  | <a href="https://reelrbtx.github.io/SMACC/melodic-devel/html/md_README.html">doxygen</a>|
| Master  | <a src="https://travis-ci.org/reelrbtx/SMACC"><img src="https://travis-ci.com/reelrbtx/SMACC.svg?branch=master"/></a> | <a href="https://reelrbtx.github.io/SMACC/master/html/md_README.html">doxygen</a>|


## Docker Containers

[![Docker Automated build](https://img.shields.io/docker/automated/pabloinigoblasco/smacc.svg?maxAge=2592000)](https://hub.docker.com/r/pabloinigoblasco/smacc/) [![Docker Pulls](https://img.shields.io/docker/pulls/pabloinigoblasco/smacc.svg?maxAge=2592000)](https://hub.docker.com/r/pabloinigoblasco/smacc/) [![Docker Stars](https://img.shields.io/docker/stars/pabloinigoblasco/smacc.svg)](https://registry.hub.docker.com/pabloinigoblasco/smacc/)


# <img src="http://smacc.ninja/wp-content/uploads/2019/07/SMACC-Logo-Pixelate-4-copy.png" width="30" align="left"/> SMACC

SMACC is an event-driven, asynchronous, behavioral state machine library for real-time ROS (Robotic Operating System) applications written in C++, designed to allow programmers to build robot control applications for multicomponent robots, in an intuitive and systematic manner [UML State Charts](http://sce.uhcl.edu/helm/rationalunifiedprocess/process/modguide/md_stadm.htm) (AKA state machines). SMACC is inspired by the [SMACH ROS package](http://wiki.ros.org/smach) and it is built on top of [Boost StateChart library](https://www.boost.org/doc/libs/1_53_0/libs/statechart/doc/index.html).

Probably the greatest strength of SMACC is that the state machines you can develop with it are strictly based on the UML Standard. This means that you have access to a clear and thoroughly studied and known approach to describe State Machines. This may be especially important in industrial environments.

The following image shows one example of a state machine built using the UML standard and shows many of the concepts that can be implemented using SMACC:
<p align="center">
<img src="http://sce.uhcl.edu/helm/rationalunifiedprocess/process/modguide/images/md_state1.gif" width="450" align="center"/>
</p>

## Features
 *  ***Powered by ROS:*** SMACC has been developed specifically to work with ROS. It is a c++ ros package that can be imported from any end-user application package.
 *   ***C++ language:*** Until now, ROS has lacked a library to develop task-level state machines in C++. Many libraries in robotics are developed in C++ so this may help during the integration of different libraries. In industrial development contexts C++ is often over Python, so this tool may be a good choice.
 *  ***Static State Machine Checking:*** SMACC inherits this from the Boost Statechart library. This helps the developer to check the consistence of the state machine at compile time (instead of runtime). In other words, it helps you to check if your state machine is well written.
 * ***Component based architecture:*** SMACC has built-in funcionality provided inside SMACC Components that can be dynamically imported at runtime and stored in the local machine. The states only access those components they are concerned with. This enables the SMACC based application to extend or improve the runtime behavior of the system.

## Canonical SMACC applications
The canonical SMACC applications are mobile robots (that may optionally have manipulators) that have to navigate around the environment and use one or more onboard tools. One example of this would be the PR2 Robot, working in a factory, navigating to some shelves with parcels, fetching them with a 6-axis arm and manipulator, and then navigating to some delivery point. Other examples would include mobile robots that must perform a systematic navigation on a designated area while operating one or more tools, such as a vacuum or angle grinder. In order to accomplish this, SMACC provides an array navigation planners that can greatly simplifiy this task (see more on section ROS Integration).

## ROS Integration

* ***Intensive use of ROS Action***. SMACC translates Action server events (Result callbacks, Feedback callbacks, etc.) into statechart events. To learn more about this, check the sections Shared Resources and SMACC Architecture.
* ***Powerful access to ROS Parameters***. Each SMACC state automatically creates a ros::NodeHandle automatically named according to the SMACC state hierarchy (see more in section Usage Examples - Ros parameters)
* ***ROS Navigation built-in funcionality***. SMACC extends the ROS navigation stack in a high level way. It provides specialized navigation planners (for the ROS Navigation Stack) that navigate only using pure spinning motions and straight motions. Implements some mechanism to perform motions recording the path and undoing them later.
These can be very useful in some industrial applications where the knowledge or certainty on the environment is higher (ros planners are focused on cluttered and dynamic environments).

## Repository Packages

This repository contains several ROS packages:

 * **smacc_core**: The core smacc library. It works as a template-based c++  header library.

 * **smacc_navigation**: a set of ros packages with some "Smacc Components" that ease the creation of high level navigation applications. The provided components remotelly control the move_base. These components helps in the creation of complex motion strategies (changing planners, recording and undoing paths, etc.)

 * **smacc_state_machine_templates**: shows a complete sample application developed with SMACC that can be reused as a canonical example of a mobile robot moving around and interacting with a custom onboard tool.

 * **smacc_interface_components**: template project that shows how an on/off tool onboard the mobile robot could be used following the SMACC methodology.

## Future Work
 * undoing paths chunks by state (store the different chunks of the path according to its state in a stack)
 * code generation based on uml diagrams
 * improving backwards planners for non linear paths
 
## Shared Resources and Shared variables

* ***Shared Action Client Resources*** Action servers in SMACC play an important role because all low-level funcionality must be located on them. In order to interact with these Action Servers SMACC provide an easy way to create shared ActionClients that can be accessed from any State. SMACC is in charge of eficently handle all the requests and send the resulting events from action servers (result messages, feedback messages, etc) to the "subscribed states" in form of statechart events. (See more about his in section internal architecture)

* ***Shared Variables*** UML statecharts basically define the high level behavior of a system. However, in practice the real state of the system may be much more complex (mesurement, environment numerical information, etc.). States usually have to share information (or comunicate to each other). In order to do that, SMACC implements a  simple but effective dictionary-based mechanism to share information (structs, objects, simple variables or pointers). (See below in tutorials: shared variable)
<p align="center">
<img src="https://github.com//brettpac/SMACC/blob/master/documentation/shared_resources.png?raw=true"  width="450" align="center"/>
</p>

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

## Executing the Radial Motion Example
This is a complete sample of a state machine that controls the motion of a simulated ridgeback mobile robot in gazebo.

This example shows how smacc::Navigation components can be used to create some systematic motion based on an state machine. The robot performs a motion pattern that plots a "Start" on the plane. The state machine in the motion is combined with the usage of some external tool "for example ground painter" that is only active on forward motions.

```
export RIDGEBACK_URDF_EXTRAS=$(rospack find radial_motion_example)/urdf/empty.xacro

roslaunch radial_motion_example radial_motion.launch
```
<p align="center">
<div style="float:left" align="center">
<img src="https://raw.githubusercontent.com/brettpac/SMACC/master/documentation/radial_motion_example.png" width="400"/>
</div>
<div style="float:left" align="center">
<img src="https://raw.githubusercontent.com/brettpac/SMACC/master/documentation/radial_motion_example_statechart.png" width="400"/>
</div>
</p>


# Tutorial
SMACC states inherits from boost::statechart:State so that you can learn the full potential of SMACC states also diving in the statechart documentation. However, the following examples briefly show how you create define SMACC states and how you would usually use them.

## Code a minimal SMACC StateMachine
In this initial example we will implement a simple state machine with a single state state that executes something at state entry and at state exit. That state machine is described in the following image:

<p align="center">
<img src="https://raw.githubusercontent.com/brettpac/SMACC/master/documentation/simpleStateMachine.png" width="450"/>
</p>

SMACC StateMachines and SmaccStates are based on the c++ [Curiously recurring template pattern](https://en.wikipedia.org/wiki/Curiously_recurring_template_pattern) so that the syntax may be strange for some developers but you will notice that it is very easy to follow. The advantage of using this kind of c++ pattern is that the definition of the state machine is correctly written.

The following chunk of code shows the minimal SMACC State machine you can create:
```cpp
#include <smacc/smacc_state_machine_base.h>

struct SimpleStateMachine
    : public SmaccStateMachineBase<SimpleStateMachine,ToolSimpleState>
{
  using SmaccState::SmaccState;
  void onEntry()
  {
  }
};
```

Every SMACC State Machine must inherit from SmaccStateMachineBase. The first template parameter is the derived class type (SimpleStateMachine itself) according to the curiosuly recurring template pattern, and the second template parameter (ToolSimpleState) would be the class name of the initial Smacc State

## Code a simple SMACC State

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

## Creating/accessing to SmaccComponents

Accessing to SMACC componets resources is one of the most important capabilities that SMACC provides. This example shows how to access to these resources form states.

For example, in this case we will asume we are in a state that controls the navigation of the vehicle, and it needs to access to the Ros Navigation Stack Action client and navigate to some position in the environment.

<p align="center">
<img src="https://raw.githubusercontent.com/brettpac/SMACC/master/documentation/shared_resource.png" width="450"/>
</p>

The code would be the following:

```cpp
struct Navigate : SmaccState<Navigate, SimpleStateMachine> 
{

public:
  // This is the smacc component (it basically is a wrapper of the ROS Action Client for move base), please check the SMACC
  // code to see how to implement your own action client
  smacc::SmaccMoveBaseActionClient *moveBaseClient_;

  using SmaccState::SmaccState;
  void onEntry()
  {
    ROS_INFO("Entering Navigate");

    // this substate will need access to the "MoveBase" resource or plugin. In this line
    // you get the reference to this resource.
    this->requiresComponent(moveBaseClient_);
    goToEndPoint();
  }

  // auxiliar function that defines the motion that is requested to the move_base action server
  void goToEndPoint() {
    geometry_msgs::PoseStamped radialStartPose = createInitialPose();

    smacc::SmaccMoveBaseActionClient::Goal goal;
    goal.target_pose.header.stamp = ros::Time::now();

    goal.target_pose = radialStartPose;
    goal.target_pose.pose.position.x = 10;
    goal.target_pose.pose.position.y = 10;
    goal.target_pose.pose.orientation =
        tf::createQuaternionMsgFromRollPitchYaw(0, 0, M_PI);

    moveBaseClient_->sendGoal(goal);
  }
};
```


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

## Add custom code on Action Result Events

In the following example, we want to add some code to the transition between the source state "Navigate" and the
destination state "ExecuteToolState". This code may be any desired custom code (for example some transition guard).
This code is located in the react method

<p align="center">
<img src="https://raw.githubusercontent.com/brettpac/SMACC/master/documentation/custom_reaction.png" width="450"/>
</p>

The following would be the code for this state machine:

```cpp
struct Navigate : SmaccState<Navigate, SimpleStateMachine> 
{
public:

  // With this line we specify that we are going to react to any EvActionResult event
  // generated by SMACC when the action server provides a response to our request
  typedef mpl::list<sc::custom_reaction<EvActionResult<SmaccMoveBaseActionClient::Result>>> reactions;
  using SmaccState::SmaccState;
  void onEntry()
  {
   [...]
  }

  // auxiliary function that defines the motion that is requested to the move_base action server
  void goToEndPoint() {
   [...]

  sc::result react(const EvActionResult<SmaccMoveBaseActionClient::Result> &ev)
  {
      // we only will react when the result is succeeded
      if (ev.getResult() == actionlib::SimpleClientGoalState::SUCCEEDED)
      {
        // ev.resultMessage provides access to the move_base action server result structure

        ROS_INFO("Received event to movebase: %s",ev.getResult().toString().c_str());
        return transit<ExecuteToolState>();
      }
      else
      {
        return forward_event(); // do nothing, the default behavior if you do not specify any return value
      } 
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

## Adding ROS Parameters to Smacc States

The SMACC states can be configured from the ros parameter server based on their hierarchy
and their class name. It is responsability of the user not to have two different state names at the same level (even if the namespace is distinct since the namespace is trimmed for parameters)

For example, imagine a StateMachine to move the mobile robot initially to some initial position, and then moving it to some other position relative to the initial position. You could put the navigation parameters in a ros configuration yaml file like this (and avoid hardcoding):

```yaml
MyStateMachine:
    State1: #Go to some initial position
        NavigationOrthogonalLine:
            Navigate:
                start_position_x: 3
                start_position_y: 0
    State2: #Go to some initial position
        NavigationOrthogonalLine:
            Navigate:
                initial_orientation_index: 0 # the initial index of the linear motion (factor of angle_increment_degrees)
                angle_increment_degree: 90    # the increment of angle between to linear motions
                linear_trajectories_count: 2  # the number of linear trajectories of the radial 
```

Then, the c++ code for the State MyStateMachine/State1/NavigationOrthogonalLine/Navigate could contain the following parameter reading funcionality:

```cpp
struct Navigate : SmaccState<Navigate, NavigationOrthogonalLine> 
{
public:
  using SmaccState::SmaccState;
  void onEntry()
  {
      geometry_msgs::Point p;
      param("start_position_x", p.x, 0);
      param("start_position_y", p.y, 0);
  }
}
```

The param template method reads from the parameters server delegating to the method defined ros::NodeHandle handle does but already located at the exact point in the parameter name hierarchy associated to this state. SMACC is also able have methods getParam and setParam that are delegated to ros::NodeHandle in the same way.

## Shared variables between states

This following example shows how to share a variable between two states. In the Navigate state the "angle_value" variable is set using the template method "setData". Latter in the ExecuteToolState it gets the value using the template method "getGlobalSMData".

```cpp
struct Navigate : SmaccState<Navigate, SimpleStateMachine>
{
public:
  using SmaccState::SmaccState;
  void onEntry()
  {
        double angle = M_PI;
        this->setGlobalSMData("angle_value", angle);
  }
};

struct ExecuteToolState : SmaccState<ExecuteToolState, SimpleStateMachine>
{
    using SmaccState::SmaccState;
    void onEntry()
    {
         int angle;
         this->getGlobalSMData("angle_value", angle);
    }
};
```

## Orthogonal Lines
SMACC proposes to work in different orthogonal lines: Navigation, Tool1, Tool2, etc. This example shows how you can define orthogonal lines in your SMACC code.
For example, we want to add two orthogonal lines: the navigation orthogonal line and the tool orthogonal line.

<p align="center">
<img src="https://raw.githubusercontent.com/brettpac/SMACC/master/documentation/orthogonal_lines.png" width="450"/>
</p>

First we will define the NavigationOrthogonal line line with a simple ToolSubState:

```cpp
struct NavigationOrthogonalLine
    : SmaccState<NavigationOrthogonalLine, SimpleStateMachine::orthogonal<0>, NavigateSubstate> {
  using SmaccState::SmaccState;
  void onEntry()                 
  {
    ROS_INFO("Entering in the navigation orthogonal line");
  }

  ~NavigationOrthogonalLine() 
  {
    ROS_INFO("Finishing the navigation orthogonal line");
  }
};

struct NavigateSubstate : SmaccState<NavigateSubstate, NavigationOrthogonalLine> 
{
    using SmaccState::SmaccState;
    void onEntry()
    {
    }
};

```
First we will define the ToolOthogonal line with a simple ToolSubState:

```cpp
struct ToolOrthogonalLine
    : SmaccState<ToolOrthogonalLine, SimpleStateMachine::orthogonal<1>, ToolSubstate> {
public:
  using SmaccState::SmaccState;
  void onEntry()          
  {
    ROS_INFO("Entering in the tool orthogonal line");
  }

  ~ToolOrthogonalLine() 
  {
    ROS_INFO("Finishing the tool orthogonal line"); 
  }
};

struct ToolSubstate : SmaccState<ToolSubstate, ToolOrthogonalLine> 
{
    using SmaccState::SmaccState;
    void onEntry()
    {
    }
};

```

