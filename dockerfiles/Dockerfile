# use ubuntu xenial, which is what the turtlebot2 people
# seemed to have been using, at least to some extent
# (it's at least the OS, I believe, that's put on the
# robot?)
FROM ubuntu:16.04

# Copy the context of our build to the app directory, at
# the root.
COPY . /app


# NOTE: we're going to use ros bouncy, which doesn't come pre-built
#       for xenial, so we have to build it.
#       Build instructions:
#       https://index.ros.org/doc/ros2/Installation/Crystal/Linux-Development-Setup/

# ANOTHER NOTE: btw, a lot of the commands used in the webpage above
#               use sudo, but docker runs everything as sudo by default
#               so it isn't needed in your RUN statement

# Install locales, which is needed for
# first step of those build instructions
RUN apt-get clean && apt-get update && apt-get install -y locales

# NOTE: all-caps comments (other than NOTE: ...) refer to
# specific sections of the build setup, and comments with more
# hashtags preceding them refer to nested subsections within that
# section.

# SYSTEM SETUP
## SET LOCALE
RUN locale-gen en_US.UTF-8

RUN update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

# (not sure if this is needed...You're technically supposed to do
# RUN export LANG=en_US.UTF-8
# but this won't actually export it. But this ENV statement isn't actually used.)
ENV LANG=en_US.UTF-8


## ADD THE ROS 2 APT REPOSITORY
# NOTE: instructions for this subsection only are specifically from
#       the following webpage:
#       https://index.ros.org/doc/ros2/Installation/Crystal/Linux-Install-Debians/#linux-install-debians-setup-sources
RUN apt-get update && \
      apt-get -y install curl gnupg2 lsb-release
# continued below


# add the ROS apt repository, which I added since it wasn't very happy otherwise
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
RUN apt-key adv --keyserver hkp://pool.sks-keyservers.net --recv-key 0xB01FA116
RUN apt-get update

# begin continuation
RUN curl http://repo.ros2.org/repos.key | apt-key add -
# pause continuation

ENV CHOOSE_ROS_DISTRO bouncy
ENV ROS1_DISTRO melodic

# unpause continuation
RUN sh -c 'echo "deb [arch=amd64,arm64] http://packages.ros.org/ros2/ubuntu `lsb_release -cs` main" > /etc/apt/sources.list.d/ros2-latest.list'
# end continuation

# returning to the build instructions...

## INSTALL DEVELOPMENT TOOLS AND ROS TOOLS
RUN apt-get update && apt-get install -y \
      build-essential \
      cmake \
      git \
      python3-colcon-common-extensions \
      python3-lark-parser \
      python3-pip \
      python3-rosdep \
      python3-vcstool \
      wget

# install pip packages for testing
RUN python3 -m pip install -U \
      argcomplete \
      flake8 \
      flake8-blind-except \
      flake8-builtins \
      flake8-comprehensions \
      flake8-deprecated \
      flake8-docstrings \
      flake8-import-order \
      flake8-quotes \
      pytest-repeat \
      pytest-rerunfailures \
      pytest \
      pytest-cov \
      pytest-runner \
      setuptools

# install FAST-RTPS dependencies <----- (?????? this was just a
#                                        comment I copied from
#                                        the build
#                                        instructions...)
RUN apt install --no-install-recommends -y \
      libasio-dev \
      libtinyxml2-dev

# GET ROS 2 CODE
# INSTALL DEPENDENCIES USING ROSDEP
RUN mkdir -p ~/ros2_ws/src && \
    ls && \
    cp /app/con/melodic.repos ~/ros2_ws && \
    cd ~/ros2_ws && wget https://raw.githubusercontent.com/ros2/ros2/release-latest/ros2.repos && vcs import src < ros2.repos && \
    rosdep init && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src --rosdistro bouncy -y --skip-keys "console_bridge fastcdr fastrtps libopensplice67 libopensplice69 python3-lark-parser rti-connext-dds-5.3.1 urdfdom_headers"

RUN ls ~/

#RUN cd ~/ros2_ws && \
#    python3 -m pip install -U lark-parser && \
#    wget https://raw.githubusercontent.com/ros2/turtlebot2_demo/bouncy/turtlebot2_demo.repos && \
#    vcs import src < turtlebot2_demo.repos && \
    #wget https://raw.githubusercontent.com/ros/rosdistro/master/melodic/distribution.yaml -O melodic.repos && \
#    vcs import src < melodic.repos
RUN apt-get install --no-install-recommends -y \
      libboost-iostreams-dev libboost-regex-dev \
      libboost-system-dev libboost-thread-dev \
      libceres-dev libgoogle-glog-dev liblua5.2-dev \
      libpcl-dev libprotobuf-dev libsdl1.2-dev \
      libsdl-image1.2-dev libsuitesparse-dev \
      libudev-dev libusb-1.0-0-dev \
      libyaml-cpp-dev protobuf-compiler \
      python-sphinx \


# so instead of the following junk we're going to follow what I've
# found at the following webpage:
# http://wiki.ros.org/melodic/Installation/Source
# to install ros-melodic

# PREREQUISITES
## INSTALLING BOOTSTRAP DEPENDENCIES
RUN apt-get install python-rosdep python-rosinstall-generator \
    python-wstool python-rosinstall build-essential

## INITIALIZING ROSDEP
RUN rosdep init
RUN rosdep update

# INSTALLATION
## CREATE A CATKIN WORKSPACE
RUN mkdir ~/ros1_catkin_ws && \
    cd ~/ros1_catkin_ws && \
    rosinstall_generator ros_comm --rosdistro melodic --deps --tar > melodic-ros_comm.rosinstall && \
    wstool init -j8 src melodic-ros_comm.rosinstall && \
    rosdep install --from-paths src --ignore-src --rosdistro melodic -y && \
    ./src/catkin/bin/catkin_make_isolated --install -DCMAKE_BUILD_TYPE=Release
#   ^^^^^^^^^^^^^^^^^ it's this last step that fails :/
# it doesn't like something about rosbag_storage

# BEGIN JUNK #
#      ros-$ROS1_DISTRO-catkin \
#      ros-$ROS1_DISTRO-kobuki-driver \
#      ros-$ROS1_DISTRO-kobuki-ftdi && \
#    colcon build --symlink-install --packages-skip cartographer cartographer_ros \
#      cv_bridge opencv_tests ros1_bridge \
#      turtlebot2_amcl turtlebot2_drivers turtlebot2_follower \
#      turtlebot2_cartographer turtlebot2_teleop vision_opencv && \
#    source /opt/ros/$ROS1_DISTRO/setup.bash && \
#    colcon build --symlink-install --packages-select cartographer cartographer_ros \
#    turtlebot2_amcl turtlebot2_cartographer turtlebot2_drivers turtlebot2_follower \
#    turtlebot2_teleop
# END JUNK #

# The following are mostly commented out because I put their code somewhere else
### START OF COMMENTED OUT CODE ###
#RUN apt-get -y install python3-colcon-common-extensions

#RUN apt-get update && \
#      export DEBIAN_FRONTEND=noninteractive && \
#RUN apt-get update && \
#      apt-get -y install ros-$CHOOSE_ROS_DISTRO-ros-base

# RUN apt-get -y install python3-argcomplete

# RUN ls /opt/ros/

# RUN source /opt/ros/$CHOOSE_ROS_DISTRO/setup.bash

# RUN export ROS1_DISTRO=$ROS1_DISTRO_NAME

# RUN apt-get -y install ros-bouncy-turtlebot2* ros-${ROS1_DISTRO}-kobuki-ftdi
### END OF COMMENTED OUT CODE ###

# install sudo
RUN apt-get update && \
      apt-get -y install sudo

# Add a user called docker, with password docker,
# that we will be using (just so that it's easier
# to copy over the commands from the internet)
RUN useradd -m docker && echo "docker:docker" | chpasswd && adduser docker sudo

# Set the user that we enter ubuntu as to be docker
USER docker

# fire up the command line
CMD /bin/bash