FROM ros:kinetic-robot
ENV PATH /usr/local/nvidia/bin:${PATH}
ENV LD_LIBRARY_PATH /usr/local/nvidia/lib:/usr/local/nvidia/lib64:${LD_LIBRARY_PATH}

ENV HOME /root
ENV DEBIAN_FRONTEND noninteractive

RUN apt-get update && apt-get install -y \
    ros-kinetic-robot=1.3.2-0* \
&& rm -rf /var/lib/apt/lists/*

# SYSTEM DEPENDENCIES
#----------------------------------------------------------
RUN export DEBIAN_FRONTEND="noninteractive"; apt-get update && apt-get install -y apt-utils && apt-get install -y git python-catkin-tools

RUN apt-get update

# DOWNLOAD MAIN REPOSITORY
#----------------------------------------------------------
RUN echo "downloading smacc repo"
RUN git clone https://github.com/reelrbtx/SMACC.git /root/src/SMACC
RUN git clone https://github.com/reelrbtx/SMACC_Viewer.git /root/src/SMACC_Viewer

RUN apt-get install -y python-argcomplete python-bloom dh-make

# INSTALL PACKAGE CLOUD SOFTWARE
# --------------------------------------------------------------
RUN apt-get -y install ruby-dev nano
RUN gem install rake
RUN gem install package_cloud

WORKDIR /root

# BUILD SMACC AND SMACC_VIEWER
# -----------------------------------------------------------------
RUN bash -c "source /opt/ros/kinetic/setup.bash; cd /root; rosdep install --from-paths src --ignore-src -r -y;"
RUN bash -c "source /opt/ros/kinetic/setup.bash; cd /root; catkin build"


# BUILD DEBIAN FILES
# ------------------------------------------------------------------------
#ADD token_file.json /root/.packagecloud
RUN echo "yaml file:/root/src/smacc_ci/rosdep.yaml" > /etc/ros/rosdep/sources.list.d/50-my-packages.list 
RUN rosdep update

RUN echo "user: $PACKAGE_CLOUD_USER"
RUN echo "token: $PACKAGE_CLOUD_TOKEN"

#ENTRYPOINT ["/bin/bash"]
WORKDIR /root
ENTRYPOINT ["cd /bash -c 'source devel/setup.bash; src/smacc_ci/generate_debs.py -smacc_src_folder src/SMACC -smacc_viewer_src_folder src/SMACC_Viewer']