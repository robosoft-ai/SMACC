#!/bin/bash
echo "build docker image with github user: $1"
sudo docker build --build-arg GITHUB_USER=$1 --build-arg GITHUB_TOKEN=$2 --build-arg PACKAGE_CLOUD_USER=$3 --build-arg PACKAGE_CLOUD_TOKEN=$4 -t package_cloud_tool_docker .