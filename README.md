# SMACC
SMACC is a ROS/C++ library to implement in easy and systematic way [UML State Charts](http://sce.uhcl.edu/helm/rationalunifiedprocess/process/modguide/md_stadm.htm) (AKA state machines). SMACC is inspired in the [SMACH ROS package](http://wiki.ros.org/smach) and it built on top of [Boost StateChart library](https://www.boost.org/doc/libs/1_53_0/libs/statechart/doc/index.html).

Probably the best streght of SMACC is that the statechart you can develop with it are strictly based on the UML Standard. This means that you can access to a clear and thoroughly studied and known approach to describe State Machines. This may be specially important on industrial environments.

[[StateChart image]](http://sce.uhcl.edu/helm/rationalunifiedprocess/process/modguide/md_stadm.htm)

## Features
 *  ***Powered by ROS:*** SMACC has been developed specifically to work with ROS. It is a c++ ros package that can be imported from any end-user application package.
 *   ***C++ language:*** ROS lacked the existence of a library to develop task-level state machine in c++. Many libraries in robotics are developed in c++ so that this may help during the integration of different libraries. In industrial development context are sometimes prefered the usage of c++ over Python, so that this tool may be a good choice.
 *  ***Static State Machine Checking:*** SMACC inherits this from the statechart library. This helps the developer to check the consistence of the state machine in compile time (instead of runtime). In other words, it helps you to check if your state machine is well written.
 * ***Dynamically extensible funcionality:*** SMACC uses rosplugins to provide rich action clients that can be dinamically imported at runtime and stored in the local machine. This enables the SMACC application extend or improve the runtime behavior of the system.  You can just replace some plugin components and keeping the original SMACC application without needing to recompile.

## Cannonical SMACC applications
The cannonical SMACC applications are mobile robots (that may optionally have manipulators) that have to navigate around the environment and use some of the onboard tools. One example could be the PR2 Robot working in a factory navigating to some selfs with parcels and fetching them to some delivery point.

Other example would be vaccum cleaner or other mobile robot that must perform a sistematic navigation on the ground executing some tool. SMACC provides some navigation planners (for the ROS Navigation Stack) that navigate only using pure spinning motions and stright motions. These can be very useful in some industrial applications where the knowledge or certainty on the environment is higher (ros planners are focused on cluttered and dynamic environments).

## Shared State Resources


## Development methodology
SMACC also defines a development methodology where State Machine nodes only contains the task-level logic, that is, the high level behavior of the robot system in some specific application.

SMACC applications have low level coupling with other software components of the robot system. SMACC code is recomended to interact with the rest of components the robot system via ROS Action Servers and **Smacc Action Plugins**.

The proposed methdology split the states into 2 or more statechart orthogonal lines that comunicate to each other via events. The orthogonal line 0 is tipically for the mobile robot navigation. The second orthogonal line and ahead are used for tools (manipulators, grippers or other custom tools).

![Diagram](http://smacc.ninja/wp-content/uploads/2018/09/SMACC-Containers-2.jpg)

## Internal Architecture

![Diagram](http://smacc.ninja/wp-content/uploads/2018/09/SMACC-Node-Map-2-2-1.jpg)


## Integration with ROS the Navigation Stack

## Repository Packages

This repository contains several ROS packages:

 * **smacc**: The core smacc library. It works as a template-based c++  header library.

 * **smacc_navigation_plugin**: it implements a smacc-actionclient-plugin to handle the move_base node remotely.

 * **smacc_reelrbtx_plugin**: it implements a smacc-actionclien-plugin to handle the reel robotics reel device.

 * **radial_motion_example**: shows a complete sample application developed with SMACC that can be reused as a canonical example of a mobile robot moving around and interacting with a custom onboard tool.

 * **smacc_tool_plugin_template**: template project that shows how an on/off tool onboard the mobile robot could be used following the SMACC methodology.

## Executing the Radial Motion Example
Requires: reelrbtx_control and reelrbtx_navigation packages and ridgeback packages

```
export RIDGEBACK_URDF_EXTRAS=$(rospack find reelrbtx_description)/urdf/reelrbtx.urdf.xacro

roslaunch radial_motion_example radial_motion.launch
```
## Tutorial
SMACC states inherits from boost::statechart:State so that you can learn the full potential of SMACC states also diving in the statechart documentation. However, the following examples briefly show how you create define SMACC states and how you would usually use them.

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

### Adding ROS Parameters to Smacc States
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
  // This is the substate constructor. This code will be executed when the
  // workflow enters in this substate (that is according to statechart the moment when this object is created)
  Navigate(my_context ctx)
    : SmaccState<Navigate, NavigationOrthogonalLine>(ctx) // call the SmaccState base constructor 
  {
      geometry_msgs::Point p;
      param("start_position_x", p.x, 0);
      param("start_position_y", p.y, 0);
  }
}

The param template method reads from the parameters server delegating to the method defined ros::NodeHandle handle does but already located at the exact point in the parameter name hierarchy associated to this state. SMACC is also able have methods getParam and setParam that are delegated to ros::NodeHandle in the same way.

```