#!/bin/bash

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"

ROS_VERSION_NAME=$1
SOURCE_FOLDER=$2
UBUNTU_VERSION=$3
PACKAGE_CLOUD_TOKEN=$4
PACKAGE_CLOUD_USER=$5

echo "ROS_VERSION_NAME: $ROS_VERSION_NAME"
echo "SOURCE_FOLDER: $SOURCE_FOLDER"
echo "PACKAGE_CLOUD_TOKEN: $PACKAGE_CLOUD_TOKEN"
echo "PACKAGE_CLOUD_USER: $PACKAGE_CLOUD_USER"
echo "UBUNTU_VERSION: $UBUNTU_VERSION"

python $SCRIPT_DIR/generate_debs.py -repo_name smacc -ros_version=$ROS_VERSION_NAME -src_folder=$SOURCE_FOLDER -token=$PACKAGE_CLOUD_TOKEN -repo_owner=$PACKAGE_CLOUD_USER -ubuntu_version=$UBUNTU_VERSION -packages smacc_msgs smacc smacc_runtime_test sr_all_events_go sr_conditional sr_event_countdown keyboard_client multirole_sensor_client ros_publisher_client ros_timer_client move_base_z_client_plugin forward_global_planner forward_local_planner backward_global_planner undo_path_global_planner backward_local_planner move_group_interface_client
