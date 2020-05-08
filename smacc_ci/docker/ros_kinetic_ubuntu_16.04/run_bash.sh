#!/bin/bash
./build.sh
xhost +
sudo docker run -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix -it smacc_kinetic_ubuntu_1604 bash
