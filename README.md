
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

SMACC is inspired by Harel's statecharts and the [SMACH ROS package](http://wiki.ros.org/smach). SMACC is built on top of the [Boost StateChart library](https://www.boost.org/doc/libs/1_53_0/libs/statechart/doc/index.html).

Probably the greatest strength of SMACC is that it offers out-of-the-box reference state machines, (found in the folder [sm_reference_library](https://github.com/reelrbtx/SMACC/tree/master/smacc_sm_reference_library))  that you can use, test, hack, and customize to quickly get your application up and running, while also knowing that the library supports advanced functionalities that are practically universal among actual working robots.

## Features
 *  ***Powered by ROS:*** SMACC has been developed specifically to work with ROS. It supports ROS topics, services and actions, right out of the box.
 *   ***Written in C++:*** Until now, ROS has lacked a library to develop task-level state machines in C++. Although libraries have been developed in scripting languages such as python, these are unsuitable for real-world industrial evironments where real-time requirements are demanded.
 *  ***Static State Machine Checking:*** SMACC inherits this from the Boost Statechart library which helps the developer to check the consistency of the state machine at compile time (instead of runtime).
 * ***Component based architecture:*** SMACC has built-in funcionality provided inside SMACC Components that can be dynamically imported at runtime and stored in the local machine. The states only access those components they are concerned with. This enables the SMACC based application to extend or improve the runtime behavior of the system.

## SMACC applications
From it's inception, SMACC was written to support the programming of multi-component, complex robots. If your project involves small, solar-powered insect robots, that simply navigate towards a light source, then SMACC might not be the right choice for you. But if you are trying to program a robot with a mobile base, a robotic arm, a gripper, two lidar sensors, a gps transceiver and an imu, then you've come to the right place. 

<p align="center">
<img src="https://github.com/reelrbtx/SMACC/blob/master/documentation/SMACC-Containers-3.jpg"  width="450" align="center"/>
</p>
 
## Getting Started
The easiest way to get started is by selecting one of the state machines in our [reference library](https://github.com/reelrbtx/SMACC/tree/master/smacc_sm_reference_library), and then hacking it to meet your needs.

Each state machine in the reference library comes with it's own README.md file, which contains the appropriate operating instructions, so that all you have to do is simply copy & paste some commands into your terminal.

