cmake_minimum_required(VERSION 2.8.3)

project(smacc_msgs)

find_package(catkin REQUIRED COMPONENTS 
  message_generation std_msgs)

add_message_files(FILES
  SmaccSMCommand.msg
  SmaccStatus.msg
  SmaccContainerInitialStatusCmd.msg
  SmaccContainerStructure.msg
  SmaccContainerStatus.msg

  SmaccState.msg
  SmaccTransition.msg
  SmaccEvent.msg
  SmaccOrthogonal.msg
  SmaccLogicUnit.msg
  SmaccStateMachine.msg
  SmaccTransitionLogEntry.msg
  )

generate_messages(DEPENDENCIES std_msgs)

catkin_package(CATKIN_DEPENDS message_runtime)