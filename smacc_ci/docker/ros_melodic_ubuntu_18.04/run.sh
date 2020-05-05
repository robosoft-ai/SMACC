#!/bin/bash
xhost +
sudo docker run -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix -it smacc_melodic_ubuntu_1804