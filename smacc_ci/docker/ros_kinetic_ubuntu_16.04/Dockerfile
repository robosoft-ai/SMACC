FROM ros:melodic-ros-base-bionic

ENV PATH /usr/local/nvidia/bin:${PATH}
ENV LD_LIBRARY_PATH /usr/local/nvidia/lib:/usr/local/nvidia/lib64:${LD_LIBRARY_PATH}

ENV HOME /root
ENV DEBIAN_FRONTEND noninteractive

# install ros packages
RUN apt-get update && apt-get install -y \
    ros-melodic-robot=1.4.1-0* \
    && rm -rf /var/lib/apt/lists/*

RUN export DEBIAN_FRONTEND="noninteractive"; apt-get update && apt-get install -y apt-utils && apt-get install -y git python-catkin-tools curl

RUN export DEBIAN_FRONTEND="noninteractive"; apt-get update && apt-get install -y apt-utils && apt-get install -y git python-catkin-tools curl
RUN curl -s  https://b0e12e65a4f16bfc4594206c69dce2a49a5eabd04efb7540:@packagecloud.io/install/repositories/reelrbtx/SMACC_viewer/script.deb.sh  | bash
RUN apt-get install -y ros-kinetic-smacc-viewer

WORKDIR /root
ENTRYPOINT bash -c "source /opt/ros/kinetic/setup.bash; roscore&  echo $DISPLAY; rosrun smacc_viewer smacc_viewer_node.py" 