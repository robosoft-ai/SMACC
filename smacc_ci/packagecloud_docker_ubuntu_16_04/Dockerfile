FROM ros:kinetic-robot
ENV PATH /usr/local/nvidia/bin:${PATH}
ENV LD_LIBRARY_PATH /usr/local/nvidia/lib:/usr/local/nvidia/lib64:${LD_LIBRARY_PATH}
ARG GITHUB_USER
ARG GITHUB_TOKEN
ARG PACKAGE_CLOUD_USER
ARG PACKAGE_CLOUD_TOKEN

ENV GITHUB_USER=$GITHUB_USER
ENV GITHUB_TOKEN=$GITHUB_TOKEN
ENV PACKAGE_CLOUD_USER=$PACKAGE_CLOUD_USER
ENV PACKAGE_CLOUD_TOKEN=$PACKAGE_CLOUD_TOKEN

RUN echo "github user: $GITHUB_USER"
RUN echo "github token: $GITHUB_TOKEN"
RUN echo "packagecloud user: $PACKAGE_CLOUD_USER"
RUN echo "packagecloud token: $PACKAGE_CLOUD_TOKEN"

ENV HOME /root
ENV DEBIAN_FRONTEND noninteractive

RUN apt-get update && apt-get install -y \
    ros-kinetic-robot=1.3.2-0* \
&& rm -rf /var/lib/apt/lists/*

# SYSTEM DEPENDENCIES
#----------------------------------------------------------
RUN export DEBIAN_FRONTEND="noninteractive"; apt-get update && apt-get install -y apt-utils && apt-get install -y git python-catkin-tools

RUN apt-get update
RUN apt-get install -y python-argcomplete python-bloom dh-make

# INSTALL PACKAGE CLOUD SOFTWARE
# --------------------------------------------------------------
RUN apt-get -y install ruby-dev nano
RUN gem install rake
RUN gem install package_cloud

# DOWNLOAD MAIN REPOSITORY
#----------------------------------------------------------
RUN echo "regen"
RUN echo "downloading smacc repo"
RUN git clone https://github.com/reelrbtx/SMACC.git /root/src/SMACC
RUN echo "downloading smacc_viewer repo with github user: ${GITHUB_USER}"
RUN git clone https://${GITHUB_USER}:${GITHUB_TOKEN}@github.com/reelrbtx/SMACC_Viewer.git /root/src/SMACC_Viewer

WORKDIR /root

RUN echo "regen"
# BUILD SMACC AND SMACC_VIEWER
# -----------------------------------------------------------------
RUN bash -c "source /opt/ros/kinetic/setup.bash; cd /root; rosdep install --from-paths src --ignore-src -r -y;"
RUN bash -c "source /opt/ros/kinetic/setup.bash; cd /root; catkin build"

# BUILD DEBIAN FILES
# ------------------------------------------------------------------------

RUN echo "yaml file:/root/src/SMACC/smacc_ci/rosdep.yaml" > /etc/ros/rosdep/sources.list.d/50-my-packages.list 
RUN rosdep update

ADD generate_debs.py /root/src/SMACC/smacc_ci/packagecloud_docker_ubuntu_16_04/generate_debs.py

WORKDIR /root
RUN bash -c 'source devel/setup.bash; python src/SMACC/smacc_ci/packagecloud_docker_ubuntu_16_04/generate_debs.py -smacc_src_folder src/SMACC -smacc_viewer_src_folder src/SMACC_Viewer -token $PACKAGE_CLOUD_TOKEN -repo_owner $PACKAGE_CLOUD_USER'
