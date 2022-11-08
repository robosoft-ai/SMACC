
ARG ROS_DOCKER_BASE
FROM $ROS_DOCKER_BASE

ARG UBUNTU_VERSION
ARG ROS_VERSION_NAME
ARG GITHUB_USER
ARG GITHUB_TOKEN
ARG PACKAGE_CLOUD_USER
ARG PACKAGE_CLOUD_TOKEN

# PYTHON_VERSION could be empty, or "3"
ARG PYTHON_VERSION=""

ENV PATH /usr/local/nvidia/bin:${PATH}
ENV LD_LIBRARY_PATH /usr/local/nvidia/lib:/usr/local/nvidia/lib64:${LD_LIBRARY_PATH}

ENV GITHUB_USER=$GITHUB_USER
ENV GITHUB_TOKEN=$GITHUB_TOKEN
ENV PACKAGE_CLOUD_USER=$PACKAGE_CLOUD_USER
ENV PACKAGE_CLOUD_TOKEN=$PACKAGE_CLOUD_TOKEN
ENV UBUNTU_VERSION=$UBUNTU_VERSION
ENV ROS_VERSION_NAME=$ROS_VERSION_NAME

RUN echo "rosversion: $ROS_VERSION_NAME"
RUN echo "pythonversion: $ROS_VERSION_NAME"
RUN echo "ubuntu version: $UBUNTU_VERSION"
RUN echo "github user: $GITHUB_USER"
RUN echo "github token: $GITHUB_TOKEN"
RUN echo "packagecloud user: $PACKAGE_CLOUD_USER"
RUN echo "packagecloud token: $PACKAGE_CLOUD_TOKEN"

ENV HOME /root
ENV DEBIAN_FRONTEND noninteractive

RUN apt-get update && apt-get -y install curl
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

RUN apt-get update && apt-get install -y \
    ros-$ROS_VERSION_NAME-robot \
&& rm -rf /var/lib/apt/lists/*

# SYSTEM DEPENDENCIES
#----------------------------------------------------------
RUN export DEBIAN_FRONTEND="noninteractive"; apt-get update && apt-get install -y apt-utils && apt-get install -y git fakeroot

RUN apt-get update
RUN apt-get upgrade -y
RUN apt-get install -y python$PYTHON_VERSION-argcomplete python$PYTHON_VERSION-bloom dh-make

# INSTALL PACKAGE CLOUD SOFTWARE
# --------------------------------------------------------------
RUN apt-get -y install ruby-dev nano
RUN gem install rake
RUN gem install package_cloud

# DOWNLOAD MAIN REPOSITORY
#----------------------------------------------------------
RUN echo "regen"
RUN echo "downloading smacc repo: "
RUN echo "branch name: $ROS_VERSION_NAME"
RUN git clone --single-branch --branch $ROS_VERSION_NAME-devel https://github.com/robosoft-ai/SMACC.git /root/src/SMACC
WORKDIR /root/src/SMACC
RUN git submodule init
RUN git submodule update

WORKDIR /root/src
RUN echo "downloading smacc_viewer repo with github user: ${GITHUB_USER}"
RUN git clone https://github.com/robosoft-ai/robosoft-ai_msgs.git /root/src/robosoft-ai_msgs
WORKDIR /root


#workaround for noetic, that should work just with rosdep
RUN apt-get install -y libpcl-dev ros-$ROS_VERSION_NAME-lms1xx ros-$ROS_VERSION_NAME-pcl-ros
#RUN echo "regen"
# BUILD SMACC
# -----------------------------------------------------------------
RUN bash -c "source /opt/ros/$ROS_VERSION_NAME/setup.bash; cd /root; rosdep install --from-paths src --ignore-src -r -y; "

RUN apt-get install -y libpcl-dev ros-$ROS_VERSION_NAME-pcl-ros
RUN bash -c "source /opt/ros/$ROS_VERSION_NAME/setup.bash; cd /root; catkin_make -j 1"

# BUILD DEBIAN FILES
# ------------------------------------------------------------------------

RUN echo "yaml file:/root/src/SMACC/smacc_ci/rosdep_${ROS_VERSION_NAME}.yaml" > /etc/ros/rosdep/sources.list.d/50-my-packages.list
RUN rosdep update


WORKDIR /root
RUN echo "regen1"
ADD generate_debs.py /root/src/SMACC/smacc_ci/packagecloud_docker/generate_debs.py
ADD generate_smacc_debs.bash /root/src/SMACC/smacc_ci/packagecloud_docker/generate_smacc_debs.bash

WORKDIR /root
RUN echo "... ROS_VERSION: $ROS_VERSION_NAME"
RUN echo "... UBUNTU_VERSION, $UBUNTU_VERSION"

RUN bash -c "source devel/setup.bash; ./src/SMACC/smacc_ci/packagecloud_docker/generate_smacc_debs.bash $ROS_VERSION_NAME src $UBUNTU_VERSION $PACKAGE_CLOUD_TOKEN $PACKAGE_CLOUD_USER"
#RUN bash -c 'source devel/setup.bash; python$PYTHON_VERSION  src/SMACC/smacc_ci/packagecloud_docker/generate_debs.py -ros_version="$ROS_VERSION_NAME" -src_folder="src/SMACC" -token="$PACKAGE_CLOUD_TOKEN" -repo_owner="$PACKAGE_CLOUD_USER" -ubuntu_version="$UBUNTU_VERSION"'
