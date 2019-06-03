# APPARENTLY the ROS docker images are already built on
# top of the official Ubuntu docker image, so you
# don't need to actually say FROM ubuntu:18.04
# Idk what version of ubuntu is included here though
FROM ros:melodic

# Copy the context of our build to the app directory, at
# the root.
COPY . /app


# install sudo
RUN apt-get update && \
      apt-get -y install sudo

# Add a user called docker, with password docker,
# that we will be using (just so that it's easier
# to copy over the commands from the internet)
RUN useradd -m docker && echo "docker:docker" | chpasswd && adduser docker sudo

# This is just so that we can use sudo without having to give a password
ADD ./sudoers.txt /etc/sudoers

RUN chmod 440 /etc/sudoers

# Set the user that we enter ubuntu as to be docker
# NOTE: this also makes it so that we actually run all of
#       the following commands as the user docker
USER docker

# Install locales, which is needed for
# first step of those build instructions
RUN sudo apt-get clean && sudo apt-get update && sudo apt-get install -y locales

# NOTE: all-caps comments (other than NOTE: ...) refer to
# specific sections of the build setup, and comments with more
# hashtags preceding them refer to nested subsections within that
# section.

# NOTE: instructions are specifically from
#       the following webpage:
#       https://index.ros.org/doc/ros2/Installation/Crystal/Linux-Install-Debians/#linux-install-debians-setup-sources

# SYSTEM SETUP
## SET LOCALE
RUN sudo locale-gen en_US.UTF-8

RUN sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

# (not sure if this is needed...You're technically supposed to do
# RUN export LANG=en_US.UTF-8
# but this won't actually export it. But this ENV statement isn't actually used.)
ENV LANG=en_US.UTF-8


# SETUP SOURCES
RUN sudo apt-get update && \
      sudo apt-get -y install curl gnupg2 lsb-release

RUN sudo curl http://repo.ros2.org/repos.key | sudo apt-key add -
RUN sudo sh -c 'echo "deb [arch=amd64,arm64] http://packages.ros.org/ros2/ubuntu `lsb_release -cs` main" > /etc/apt/sources.list.d/ros2-latest.list'

# INSTALL ROS2 PACKAGES
ENV CHOOSE_ROS_DISTRO bouncy

RUN sudo apt-get update && \
    sudo apt-get install -y ros-$CHOOSE_ROS_DISTRO-ros-base

RUN /bin/bash -c "source /opt/ros/$CHOOSE_ROS_DISTRO/setup.bash"

RUN . /opt/ros/$CHOOSE_ROS_DISTRO/setup.sh && \
    sudo apt-get update && \
    sudo apt-get install -y ros-$CHOOSE_ROS_DISTRO-ros1-bridge

# install wget, because it wasn't installed
RUN sudo apt-get update && \
    sudo apt-get install -y wget

# INSTALL DEVELOPMENT TOOLS AND ROS TOOLS
RUN sudo apt-get clean && \
  sudo apt-get update && sudo apt-get install -y \
  build-essential \
  cmake \
  git \
  python3-colcon-common-extensions \
  python3-lark-parser \
  python3-pip \
  python-rosdep \
  python3-vcstool \
  wget
# install some pip packages needed for testing
RUN python3 -m pip install -U \
  argcomplete \
  flake8 \
  flake8-blind-except \
  flake8-builtins \
  flake8-class-newline \
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
# install Fast-RTPS dependencies
RUN sudo apt install --no-install-recommends -y \
  libasio-dev \
  libtinyxml2-dev


# note: all following section headings are from turtlebot2_demo readme
# INSTALLATION
## INSTALLATION FROM SOURCE
RUN mkdir -p ~/ros2_ws/src && \
    cd ~/ros2_ws && \
    wget https://raw.githubusercontent.com/ros2/turtlebot2_demo/bouncy/turtlebot2_demo.repos && \
    vcs import src < turtlebot2_demo.repos

### INSTALL SOME DEPENDENCIES
ENV ROS1_DISTRO melodic

RUN sudo apt-get clean && \
    sudo apt-get update && \
    sudo apt-get install --no-install-recommends -y libboost-iostreams-dev libboost-regex-dev libboost-system-dev libboost-thread-dev libceres-dev libgoogle-glog-dev liblua5.2-dev libpcl-dev libprotobuf-dev libsdl1.2-dev libsdl-image1.2-dev libsuitesparse-dev libudev-dev libusb-1.0-0-dev libyaml-cpp-dev protobuf-compiler python-sphinx ros-$ROS1_DISTRO-catkin ros-$ROS1_DISTRO-kobuki-driver ros-$ROS1_DISTRO-kobuki-ftdi


### BUILD THE ROS2 CODE

# note: does this seem inefficient? yes. yes it does.
#       however, it's the only thing that's kind of
#       worked thus far...at least in the command line.
#       I'd be down to try seeing if I could omit any steps,
#       but honestly the build takes so long anyway, that trying to
#       eliminate 

#    . /opt/ros/bouncy/setup.sh && \


RUN cd ~/ros2_ws && \
    sudo rosdep init || true && \
    rosdep update && \
    echo "Rosdep installing" && \
    rosdep install --from-paths src \
                   --ignore-src \
                   --rosdistro $CHOOSE_ROS_DISTRO \
                   -y -r \
                   --skip-keys "console_bridge fastcdr fastrtps libopensplice67 rti-connext-dds-5.3.1 urdfdom_headers"


RUN cd ~/ros2_ws && \
    . /opt/ros/$CHOOSE_ROS_DISTRO/setup.sh && \
    colcon build --symlink-install --packages-skip \
      cartographer cartographer_ros \
      cv_bridge opencv_tests \
      ros1_bridge turtlebot2_amcl \
      turtlebot2_drivers turtlebot2_follower \
      turtlebot2_cartographer turtlebot2_teleop \
      vision_opencv turtlebot2_demo && \
    echo "colcon build 1 stopped" && \
    . /opt/ros/$ROS1_DISTRO/setup.sh && \
    colcon build --symlink-install --packages-select cartographer cartographer_ros turtlebot2_amcl turtlebot2_cartographer turtlebot2_drivers turtlebot2_follower turtlebot2_teleop || true && \
    { rosdep init || true; } && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src --rosdistro $ROS1_DISTRO -y -r --skip-keys "console_bridge fastcdr fastrtps libopensplice67 rti-connext-dds-5.3.1 urdfdom_headers" && \
    colcon build --symlink-install --packages-select cartographer cartographer_ros turtlebot2_amcl turtlebot2_cartographer turtlebot2_drivers turtlebot2_follower turtlebot2_teleop && \
    . /opt/ros/$CHOOSE_ROS_DISTRO/setup.sh && \
    colcon build --symlink-install --packages-skip \
      cartographer cartographer_ros \
      cv_bridge opencv_tests \
      ros1_bridge turtlebot2_amcl \
      turtlebot2_drivers turtlebot2_follower \
      turtlebot2_cartographer turtlebot2_teleop \
      vision_opencv

#    sudo rosdep init && \


# CONFIGURE A COUPLE OF THINGS
## SETUP THE UDEV RULES
### COPY THE ASTRA UDEV RULES
#RUN wget https://raw.githubusercontent.com/ros2/ros_astra_camera/ros2/56-orbbec-usb.rules #&& \
#    sudo cp 56-orbbec-usb.rules /etc/udev/rules.d

### COPY THE KOBUKI UDEV RULE
#RUN sudo cp `rospack find kobuki_ftdi`/57-kobuki.rules /etc/udev/rules.d

#RUN sudo service udev reload && \
#    sudo service udev restart


## SOURCE YOUR WORKSPACE
# note: this may be incorrect because I installed -- i think -- ros2
#       from debians but installed the turtlebot2_demo from source,
#       but this command is for if you install from binaries.
#       However, doing any sourcing at all actually does not affect
#       the resulting image, apparently, which seems a bit silly.
#RUN /bin/bash -c "source /opt/ros/$ROS1_DISTRO/setup.bash" && \
#    /bin/bash -c "source /opt/ros/bouncy/setup.bash

# fire up the command line
CMD /bin/bash