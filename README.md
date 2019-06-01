# ros1-to-ros2-sandbox

## Build

Build with

```
docker build -t <username>/ros2turtle:<version> .
```

And run with

```
docker run -t -i <username>/ros2turtle:<version>
```

Note: the name can be changed, and is a bit arbitrary, and the
only reason why I've been using a name is because then you can
separately run and build, so this is just what I've been doing to
get my hands on this in the first place. I'm not sure what Docker
best practices are, but this seems reasonable.

I also just include a version so that if I have a working docker
image and I'm making changes, I can just increase the version number
so it won't affect the currently "working" docker image.

## Known Issues

The `Dockerfile` does actually build, but there are several lines
in it that do not build. I figured out from last time that the permission
error was probably due to trying to run the `colcon build` from the root
directory, right after entering the docker container, but I should have
actually changed to the ROS2 workspace directory, which the dockerfile
creates. So now the code that is not running in `Dockerfile` is


```
#RUN cd ~/ros2_ws && \
#    . /opt/ros/bouncy/setup.sh && \
#    colcon build --symlink-install --packages-skip cartographer cartographer_ros cv_bridge opencv_tests ros1_bridge turtlebot2_amcl turtlebot2_drivers turtlebot2_follower turtlebot2_cartographer turtlebot2_teleop vision_opencv

#RUN /bin/bash -c "source /opt/ros/$ROS1_DISTRO/setup.bash"
#RUN colcon build --symlink-install --packages-select cartographer cartographer_ros turtlebot2_amcl turtlebot2_cartographer turtlebot2_drivers turtlebot2_follower turtlebot2_teleop



#RUN wget https://raw.githubusercontent.com/ros2/ros_astra_camera/ros2/56-orbbec-usb.rules && \
#    sudo cp 56-orbbec-usb.rules /etc/udev/rules.d

#RUN sudo cp `rospack find kobuki_ftdi`/57-kobuki.rules /etc/udev/rules.d

#RUN sudo service udev reload && \
#    sudo service udev restart

#RUN /bin/bash -c "source /opt/ros/$ROS1_DISTRO/setup.bash" && \
#    /bin/bash -c "source /opt/ros/bouncy/setup.bash
```

There still seems to be something wrong with `colcon build`, and
here's the context leading up to that error.

```
Audrey:ros1-to-ros2-sandbox audrey$ docker build -t audreyseo/rosubuntu:1.0.6 .
Sending build context to Docker daemon  114.7kB
Step 1/27 : FROM ros:melodic
 ---> 9d273b71adb4
Step 2/27 : COPY . /app
 ---> 422e9fd12731
Step 3/27 : RUN apt-get update &&       apt-get -y install sudo
 ---> Running in 836e019bb272
Get:1 http://security.ubuntu.com/ubuntu bionic-security InRelease Æ88.7 kBÅ
Get:2 http://packages.ros.org/ros/ubuntu bionic InRelease Æ4031 BÅ
Get:3 http://archive.ubuntu.com/ubuntu bionic InRelease Æ242 kBÅ
...
Setting up libpcl-recognition1.8:amd64 (1.8.1+dfsg1-2ubuntu2.18.04.1) ...
Setting up libpcl-visualization1.8:amd64 (1.8.1+dfsg1-2ubuntu2.18.04.1) ...
Setting up libvtk6-java (6.3.0+dfsg1-11build1) ...
Setting up libvtk6-dev (6.3.0+dfsg1-11build1) ...
Setting up libvtk6-qt-dev (6.3.0+dfsg1-11build1) ...
Setting up libpcl-dev (1.8.1+dfsg1-2ubuntu2.18.04.1) ...
Processing triggers for libc-bin (2.27-3ubuntu1) ...
Removing intermediate container 4d4652caecad
 ---> 00337282ca26
Step 26/26 : CMD /bin/bash
 ---> Running in 1dcc2306fc62
Removing intermediate container 1dcc2306fc62
 ---> fe5f639c40c0
Successfully built fe5f639c40c0
Successfully tagged audreyseo/rosubuntu:1.0.6
Audrey:ros1-to-ros2-sandbox audrey$ docker run -t -i audreyseo/rosubuntu:1.0.6
docker@348cef85eb56:/$ cd ~/ros2_ws/
docker@348cef85eb56:~/ros2_ws$ ls
src  turtlebot2_demo.repos
docker@348cef85eb56:~/ros2_ws$ . /opt/ros/bouncy/setup.sh
ROS_DISTRO was set to 'melodic' before. Please make sure that the environment does not mix paths from different distributions.
docker@348cef85eb56:~/ros2_ws$ source /opt/ros/bouncy/setup.sh
docker@348cef85eb56:~/ros2_ws$ colcon build --symlink-install --packages-skip cartographer cartographer_ros cv_bridge opencvØ
> _tests ros1_bridge turtlebot2_amcl turtlebot2_drivers turtlebot2_follower turtlebot2_cartographer Ø
> turtlebot2_teleop vision_opencv
Æ0.788sÅ WARNING:colcon.colcon_core.package_selection:ignoring unknown package 'ros1_bridge' in --packages-skip
Starting >>> image_geometry
Starting >>> map_server
Starting >>> astra_camera
Starting >>> cartographer_ros_msgs
--- stderr: image_geometry                                                                                                
CMake Error at CMakeLists.txt:15 (find_package):
  By not providing "FindOpenCV.cmake" in CMAKE_MODULE_PATH this project has
  asked CMake to find a package configuration file provided by "OpenCV", but
  CMake did not find one.

  Could not find a package configuration file provided by "OpenCV" with any
  of the following names:

    OpenCVConfig.cmake
    opencv-config.cmake

  Add the installation prefix of "OpenCV" to CMAKE_PREFIX_PATH or set
  "OpenCV_DIR" to a directory containing one of the above files.  If "OpenCV"
  provides a separate development package or SDK, be sure it has been
  installed.


---
Failed   <<< image_geometry	Æ Exited with code 1 Å
Aborted  <<< astra_camera                                                                                                               
Aborted  <<< cartographer_ros_msgs                                                                          
Aborted  <<< map_server                                    

Summary: 0 packages finished Æ4.36sÅ
  1 package failed: image_geometry
  3 packages aborted: astra_camera cartographer_ros_msgs map_server
  1 package had stderr output: image_geometry
  8 packages not processed
docker@348cef85eb56:~/ros2_ws$
```

I thought I didn't know what to make of the last error,
but I REALLY do not know what to make of _this_ error.

The only webpages that describe anything like this (that I could find
through Google, which admittedly only sees something like 1% of
webpages) are [one](https://answers.ros.org/question/312557/ros-2-crystal-in-1604-fails-to-build-ros2_overlay_ws/), [two](https://github.com/ros2/ros2/issues/338), [three](https://answers.ros.org/question/319610/create-dockerfile-for-ros2-package-ament_cmake-error/), and [four](https://answers.ros.org/question/316447/colcon-build-fails-1804/).

In the first one, the solution was to call `source ~/ros2_ws/install/local_setup.bash`, which did not help. Additionally, the `install` directory is only created once you've already tried to run `colcon build`.

For the second, the issue was the locale, but that has already
been configured correctly, at least the last time I checked (which
was in between the last commit and this one).

The third was probably the most helpful, since it alerted me to the fact that I was running

```
#RUN colcon build --symlink-install --packages-skip cartographer cartographer_ros cv_bridge opencv_tests ros1_bridge turtlebot2_amcl turtlebot2_drivers turtlebot2_follower turtlebot2_cartographer turtlebot2_teleop vision_opencv
```

when I should have been running

```
#RUN cd ~/ros2_ws && \
#    . /opt/ros/bouncy/setup.sh && \
#    colcon build --symlink-install --packages-skip cartographer cartographer_ros cv_bridge opencv_tests ros1_bridge turtlebot2_amcl turtlebot2_drivers turtlebot2_follower turtlebot2_cartographer turtlebot2_teleop vision_opencv
```

but as you can see in the "context" that I provided above, this didn't fix anything.

The fourth says that you need to follow [the instructions for installing
dependencies](https://index.ros.org/doc/ros2/Tutorials/RQt-Source-Install/#install-dependencies),
which tells you to run

```
rosdep install --from-paths src --ignore-src --rosdistro bouncy -y --skip-keys "console_bridge fastcdr fastrtps libopensplice67 rti-connext-dds-5.3.1 urdfdom_headers"
```

When I tried to do this in the docker container, this is what I got:

```
Audrey:ros1-to-ros2-sandbox audrey$ docker run -t -i audreyseo/rosubuntu:1.0.6
docker@1c2348ea38b2:/$ cd ~/ros2_ws/
docker@1c2348ea38b2:~/ros2_ws$ ls
src  turtlebot2_demo.repos
docker@1c2348ea38b2:~/ros2_ws$ rosdep init
ERROR: default sources list file already exists:
	/etc/ros/rosdep/sources.list.d/20-default.list
Please delete if you wish to re-initialize
docker@1c2348ea38b2:~/ros2_ws$ rosdep update
reading in sources list data from /etc/ros/rosdep/sources.list.d
Hit https://raw.githubusercontent.com/ros/rosdistro/master/rosdep/osx-homebrew.yaml
Hit https://raw.githubusercontent.com/ros/rosdistro/master/rosdep/base.yaml
Hit https://raw.githubusercontent.com/ros/rosdistro/master/rosdep/python.yaml
Hit https://raw.githubusercontent.com/ros/rosdistro/master/rosdep/ruby.yaml
Hit https://raw.githubusercontent.com/ros/rosdistro/master/releases/fuerte.yaml
Query rosdistro index https://raw.githubusercontent.com/ros/rosdistro/master/index-v4.yaml
Skip end-of-life distro "ardent"
Add distro "bouncy"
Add distro "crystal"
Add distro "dashing"
Skip end-of-life distro "groovy"
Skip end-of-life distro "hydro"
Skip end-of-life distro "indigo"
Skip end-of-life distro "jade"
Add distro "kinetic"
Add distro "lunar"
Add distro "melodic"
updated cache in /home/docker/.ros/rosdep/sources.cache
docker@1c2348ea38b2:~/ros2_ws$ rosdep install --from-paths src --ignore-src --rosdistro bouncy -y --skip-keys "console_bridge fastcdr fastrtps libopensplice67 rti-connext-dds-5.3.1 urdfdom_headers"

ERROR: the following packages/stacks could not have their rosdep keys resolved
to system dependencies:
cv_bridge: Cannot locate rosdep definition for Æopencv3Å
docker@1c2348ea38b2:~/ros2_ws$
```

I wasn't sure if I need to either re-initialize by deleting
the previous default sources list, so I tried doing that.

```
Audrey:ros1-to-ros2-sandbox audrey$ docker run -t -i audreyseo/rosubuntu:1.0.6
docker@f9c421174eaf:/$ sudo rm /etc/ros/rosdep/sources.list.d/20-default.list 
docker@f9c421174eaf:/$ cd ~/ros2_ws
docker@f9c421174eaf:~/ros2_ws$ sudo rosdep init
Wrote /etc/ros/rosdep/sources.list.d/20-default.list
Recommended: please run

	rosdep update

docker@f9c421174eaf:~/ros2_ws$ rosdep update
reading in sources list data from /etc/ros/rosdep/sources.list.d
Hit https://raw.githubusercontent.com/ros/rosdistro/master/rosdep/osx-homebrew.yaml
Hit https://raw.githubusercontent.com/ros/rosdistro/master/rosdep/base.yaml
Hit https://raw.githubusercontent.com/ros/rosdistro/master/rosdep/python.yaml
Hit https://raw.githubusercontent.com/ros/rosdistro/master/rosdep/ruby.yaml
Hit https://raw.githubusercontent.com/ros/rosdistro/master/releases/fuerte.yaml
Query rosdistro index https://raw.githubusercontent.com/ros/rosdistro/master/index-v4.yaml
Skip end-of-life distro "ardent"
Add distro "bouncy"
Add distro "crystal"
Add distro "dashing"
Skip end-of-life distro "groovy"
Skip end-of-life distro "hydro"
Skip end-of-life distro "indigo"
Skip end-of-life distro "jade"
Add distro "kinetic"
Add distro "lunar"
Add distro "melodic"
updated cache in /home/docker/.ros/rosdep/sources.cache
docker@f9c421174eaf:~/ros2_ws$ rosdep install --from-paths src --ignore-src --rosdistro bouncy -y --skip-keys "console_bridge fastcdr fastrtps libopensplice67 rti-connext-dds-5.3.1 urdfdom_headers"
ERROR: the following packages/stacks could not have their rosdep keys resolved
to system dependencies:
cv_bridge: Cannot locate rosdep definition for Æopencv3Å
docker@f9c421174eaf:~/ros2_ws$ 
```

Also, I find it interesting that opencv could not have its rosdep key resolved,
considering that cmake couldn't find it, according to the original error that
doing this `rosdep install <...>` was supposed to fix.
