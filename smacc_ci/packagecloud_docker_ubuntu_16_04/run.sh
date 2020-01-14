#!/bin/bash
sudo docker run -it package_cloud_tool_docker -e PACKAGE_CLOUD_USER=$1 -e PACKAGE_CLOUD_TOKEN=$2
