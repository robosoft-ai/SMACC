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

## Tutorial

### Adding parameters to states