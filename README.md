
## Continuous Integration:


| ROS Distro  | Continuous Integration | Code Format & Quality | Documentation | Binary Packages |
| :-----------: | :-----------: | :-----------: | :-----------: | :-----------: |
| Melodic  | [![Continuous Integration](https://github.com/robosoft-ai/SMACC/actions/workflows/CI.yaml/badge.svg)](https://github.com/robosoft-ai/SMACC/actions/workflows/CI.yaml?branch=melodic-devel)| [![Code Format & Quality](https://github.com/robosoft-ai/SMACC/actions/workflows/code_quality.yml/badge.svg?branch=melodic-devel)](https://github.com/robosoft-ai/SMACC/actions/workflows/code_quality.yml) | [![Doxygen](https://github.com/robosoft-ai/SMACC/actions/workflows/doxygen.yml/badge.svg?branch=melodic-devel)](https://github.com/robosoft-ai/SMACC/actions/workflows/doxygen.yml) <br/> <a href="https://robosoft-ai.github.io/smacc_doxygen/melodic/html/namespaces.html">doxygen</a>|[![Build Status](https://build.ros.org/job/Mdev__smacc__ubuntu_bionic_amd64/badge/icon?subject=ros-buildfarm)](https://build.ros.org/job/Mdev__smacc__ubuntu_bionic_amd64/)<br/>[![bloom-release](https://github.com/robosoft-ai/SMACC/actions/workflows/bloom_release.yml/badge.svg?branch=melodic-devel)](https://github.com/robosoft-ai/SMACC/actions/workflows/bloom_release.yml)<br/>[SMACC](https://index.ros.org/p/smacc/github-robosoft-ai-smacc/#melodic)|
| Noetic  | [![Continuous Integration](https://github.com/robosoft-ai/SMACC/actions/workflows/CI.yaml/badge.svg)](https://github.com/robosoft-ai/SMACC/actions/workflows/CI.yaml) | [![Code Format & Quality](https://github.com/robosoft-ai/SMACC/actions/workflows/code_quality.yml/badge.svg?branch=noetic-devel)](https://github.com/robosoft-ai/SMACC/actions/workflows/code_quality.yml) | [![Doxygen](https://github.com/robosoft-ai/SMACC/actions/workflows/doxygen.yml/badge.svg?branch=noetic-devel)](https://github.com/robosoft-ai/SMACC/actions/workflows/doxygen.yml) <br/> <a href="https://robosoft-ai.github.io/smacc_doxygen/noetic/html/namespaces.html">doxygen</a>|  [![Build Status](https://build.ros.org/job/Ndev__smacc__ubuntu_focal_amd64/badge/icon?subject=ros-buildfarm)](https://build.ros.org/job/Ndev__smacc__ubuntu_focal_amd64/)<br/>[![bloom-release](https://github.com/robosoft-ai/SMACC/actions/workflows/bloom_release.yml/badge.svg?branch=noetic-devel)](https://github.com/robosoft-ai/smacc/actions/workflows/bloom_release.yml) <br/>[SMACC](https://index.ros.org/p/smacc/github-robosoft-ai-smacc/#noetic)|

## Docker Containers

[![Docker Automated build](https://img.shields.io/docker/automated/pabloinigoblasco/smacc.svg?maxAge=2592000)](https://hub.docker.com/r/pabloinigoblasco/smacc/) [![Docker Pulls](https://img.shields.io/docker/pulls/pabloinigoblasco/smacc.svg?maxAge=2592000)](https://hub.docker.com/r/pabloinigoblasco/smacc/) [![Docker Stars](https://img.shields.io/docker/stars/pabloinigoblasco/smacc.svg)](https://registry.hub.docker.com/pabloinigoblasco/smacc/)


# <img src="http://smacc.dev/wp-content/uploads/2019/07/SMACC-Logo-Pixelate-4-copy.png" width="30" align="left"/> SMACC

SMACC is an event-driven, asynchronous, behavioral state machine library for real-time ROS (Robotic Operating System) applications written in C++, designed to allow programmers to build robot control applications for multicomponent robots, in an intuitive and systematic manner.

SMACC was inspired by Harel's statecharts and the [SMACH ROS package](http://wiki.ros.org/smach). SMACC is built on top of the [Boost StateChart library](https://www.boost.org/doc/libs/1_53_0/libs/statechart/doc/index.html).


## Features
 *  ***Powered by ROS:*** SMACC has been developed specifically to work with ROS. It supports ROS topics, services and actions, right out of the box.
 *   ***Written in C++:*** Until now, ROS has lacked a library to develop task-level behavioral state machines in C++. Although libraries have been developed in scripting languages such as python, these are unsuitable for real-world industrial environments where real-time requirements are demanded.
 *   ***Orthogonals:*** Originally conceived by David Harel in 1987, orthogonality is absolutely crucial to developing state machines for complex robotic systems. This is because complex robots are always a collection of hardware devices which require communication protocols, start-up determinism, etc. With orthogonals, it is an intuitive and relatively straight forward exercise (at least conceptually;) to code a state machine for a robot comprising a mobile base, a robotic arm, a gripper, two lidar sensors, a gps transceiver and an imu, for instance.
 *  ***Static State Machine Checking:*** One of the features that SMACC inherits from Boost Statechart is that you get compile time validation checking. This benefits developers in that the amount of runtime testing necessary to ship quality software that is both stable and safe is dramatically reduced. Our philosophy is "Wherever possible, let the compiler do it".
 *  ***State Machine Reference Library:*** With a constantly growing library of out-of-the-box reference state machines, (found in the folder [sm_reference_library](https://github.com/robosoft-ai/SMACC/tree/master/smacc_sm_reference_library)) guaranteed to compile and run, you can jumpstart your development efforts by choosing a reference machine that is closest to your needs, and then customize and extend to meet the specific requirements of your robotic application. All the while knowing that the library supports advanced functionalities that are practically universal among actual working robots.
 *  ***SMACC Client Library:*** SMACC also features a constantly growing library of [clients](https://github.com/robosoft-ai/SMACC/tree/master/smacc_client_library) that support ROS Action Servers, Service Servers and other nodes right out-of-the box. The clients within the SMACC Client library have been built utilizing a component based architecture that allows for developer to build powerful clients of their own. Current clients of note include MoveBaseZ, a full featured Action Client built to integrate with the ROS Navigation stack, the ros_timer_client, the multi_role_sensor_client, and a keyboard_client used extensively for state machine drafting & debugging.
  *  ***Extensive Documentation:*** Although many ROS users are familiar with doxygen, our development team has spent a lot of time researching the more advanced features of doxygen such as uml style class diagrams and call graphs, and we've used them to document the SMACC library. Have a look to [our doxygen sites](https://robosoft-ai.github.io/smacc_doxygen/master/html/namespaces.html) and we think you'll be blown away at what Doxygen looks like when [it's done right](https://robosoft-ai.github.io/smacc_doxygen/master/html/classsmacc_1_1ISmaccStateMachine.html) and it becomes a powerful tool to research a codebase.
  *  ***SMACC Viewer:*** The SMACC library works out of the box with the SMACC Viewer. This allows developers to visualize and runtime debug the state machines they are working on. The SMACC Viewer is closed source, but is free and can be [installed](http://smacc.dev/smacc-viewer/) via apt-get. To view the SMACC Viewer in action, click [here](https://www.youtube.com/watch?v=WVt4M_teA5I) and [here](https://www.youtube.com/watch?v=fdy37WvC4FQ). Be sure to set the youtube video to 720p HD.


## SMACC applications
From it's inception, SMACC was written to support the programming of multi-component, complex robots. If your project involves small, solar-powered insect robots, that simply navigate towards a light source, then SMACC might not be the right choice for you. But if you are trying to program a robot with a mobile base, a robotic arm, a gripper, two lidar sensors, a gps transceiver and an imu, then you've come to the right place.


## Getting Started
The easiest way to get started is by selecting one of the state machines in our [reference library](https://github.com/robosoft-ai/SMACC/tree/master/smacc_sm_reference_library), and then hacking it to meet your needs.

Each state machine in the reference library comes with it's own README.md file, which contains the appropriate operating instructions, so that all you have to do is simply copy & paste some commands into your terminal.


  *  If you are looking for a minimal example, we recommend [sm_atomic](https://github.com/robosoft-ai/SMACC/tree/master/smacc_sm_reference_library/sm_atomic).

  *  If you are looking for a slightly more complicated, but still very simple example, try [sm_calendar_week](https://github.com/robosoft-ai/SMACC/tree/master/smacc_sm_reference_library/sm_calendar_week).

  *  If you are looking for a minimal example but with a looping superstate, try [sm_three_some](https://github.com/robosoft-ai/SMACC/tree/master/smacc_sm_reference_library/sm_three_some).

  *  If you want to get started with the ROS Navigation stack right away, try [sm_dance_bot](https://github.com/robosoft-ai/SMACC/tree/master/smacc_sm_reference_library/sm_dance_bot).

  *  If you want to get started with ROS Navigation and exploring the orthogonal read-write cycle, then try [sm_dance_bot_strikes_back](https://github.com/robosoft-ai/SMACC/tree/master/smacc_sm_reference_library/sm_dance_bot_strikes_back).


Operating instructions can be found in each reference state machines readme file.
Happy Coding.

## Support
If you are interested in getting involved or need a little support, feel free to contact us by emailing brett@robosoft.ai
