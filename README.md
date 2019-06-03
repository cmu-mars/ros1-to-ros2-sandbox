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

### `colcon build` failing only when run from Dockerfile

I've gotten the colcon build to work twice now, in this sort of hack-y fashion.

```
Audrey:ros1-to-ros2-sandbox audrey$ docker run -t -i audreyseo/rosubuntu:1.0.6
docker@f9c421174eaf:/$ sudo rm /etc/
...
docker@f9c421174eaf:/$ sudo rm /etc/r
...
docker@f9c421174eaf:/$ sudo rm /etc/ros/rosdep/sources.list.d/20-default.list 
docker@f9c421174eaf:/$ cd ~/ros2_ws
docker@f9c421174eaf:~/ros2_ws$ rosdep init
...
docker@f9c421174eaf:~/ros2_ws$ sudo rosdep init
...
docker@f9c421174eaf:~/ros2_ws$ rosdep update
...
docker@f9c421174eaf:~/ros2_ws$ rosdep install --from-paths src --ignore-src --rosdistro bouncy -y --skip-keys "console_bridge fastcdr fastrtps libopensplice67 rti-connext-dds-5.3.1 urdfdom_headers"
ERROR: the following packages/stacks could not have their rosdep keys resolved
to system dependencies:
cv_bridge: Cannot locate rosdep definition for Æopencv3Å
docker@f9c421174eaf:~/ros2_ws$ rosdep install --from-paths src --ignore-src --rosdistro bouncy -y -r --skip-keys "console_bridge fastcdr fastrtps libopensplice67 rt
...
#All required rosdeps installed successfully
docker@f9c421174eaf:~/ros2_ws$ source /opt/ros/bouncy/setup.
...
docker@f9c421174eaf:~/ros2_ws$ source /opt/ros/bouncy/setup.bash
...
docker@f9c421174eaf:~/ros2_ws$ colcon build --symlink-install --packages-skip cartographer cartographer_ros cv_bridge opencvØ
> _tests ros1_bridge turtlebot2_amcl turtlebot2_drivers turtlebot2_follower turtlebot2_cartographer Ø
> turtlebot2_teleop vision_opencv
...
Check that the following packages have been built:
- cartographer
- turtlebot2_drivers
- cartographer_ros
- turtlebot2_follower
- turtlebot2_teleop
- turtlebot2_amcl
- turtlebot2_cartographer

Finished <<< astra_camera Æ1min 26sÅ
Starting >>> turtlebot2_demo
Failed   <<< turtlebot2_demo	Æ Exited with code 1 Å

Summary: 11 packages finished Æ1min 26sÅ
  1 package failed: turtlebot2_demo
  1 package had stderr output: astra_camera
docker@f9c421174eaf:~/ros2_ws$ colcon build --symlink-install --packages-skip cartographer cartographer_ros cv_bridge opencv_tests ros1_bridge turtlebot2_amcl turtlebot2_drivers turtlebot2_follower turtlebot2_cartographer turtlebot2_teleop vision_opencv
...
Check that the following packages have been built:
- cartographer
- turtlebot2_drivers
- cartographer_ros
- turtlebot2_follower
- turtlebot2_teleop
- turtlebot2_amcl
- turtlebot2_cartographer

Finished <<< astra_camera Æ6.27sÅ
Starting >>> turtlebot2_demo
Failed   <<< turtlebot2_demo	Æ Exited with code 1 Å

Summary: 11 packages finished Æ6.60sÅ
  1 package failed: turtlebot2_demo
  1 package had stderr output: astra_camera
docker@f9c421174eaf:~/ros2_ws$ source /opt/ros/$ROS1_DISTRO/setup.bash
ROS_DISTRO was set to 'bouncy' before. Please make sure that the environment does not mix paths from different distributions.
docker@f9c421174eaf:~/ros2_ws$ colcon build --symlink-install --packages-select cartographer cartographer_ros turtlebot2_amcl turtlebot2_cartographer turtlebot2_drivers turtlebot2_follower turtlebot2_teleop
...
ÆProcessing: cartographerÅ                                       
--- stderr: cartographer                                          
WARNING: html_static_path entry u'/home/docker/ros2_ws/src/vendor/cartographer/docs/source/_static' does not exist
---
Finished <<< cartographer Æ2min 2sÅ
Starting >>> cartographer_ros
...
--- stderr: cartographer_ros                                           
c++: internal compiler error: Killed (program cc1plus)
Please submit a full bug report,
with preprocessed source if appropriate.
See <file:///usr/share/doc/gcc-7/README.Bugs> for instructions.
makeÆ2Å: *** ÆCMakeFiles/cartographer_ros.dir/cartographer_ros/node.cc.oÅ Error 4
makeÆ2Å: *** Waiting for unfinished jobs....
makeÆ1Å: *** ÆCMakeFiles/cartographer_ros.dir/allÅ Error 2
make: *** ÆallÅ Error 2
---
Failed   <<< cartographer_ros	Æ Exited with code 2 Å

Summary: 5 packages finished Æ18min 46sÅ
  1 package failed: cartographer_ros
  2 packages had stderr output: cartographer cartographer_ros
  1 package not processed
docker@f9c421174eaf:~/ros2_ws$ 
docker@f9c421174eaf:~/ros2_ws$ rosdep init
ERROR: default sources list file already exists:
	/etc/ros/rosdep/sources.list.d/20-default.list
Please delete if you wish to re-initialize
docker@f9c421174eaf:~/ros2_ws$ rosdep update
...
docker@f9c421174eaf:~/ros2_ws$ colcon build --symlink-install --packages-select cartographer cartographer_ros turtlebot2_amcl turtlebot2_cartographer turtlebot2_drivers turtlebot2_follower turtlebot2_teleop
Starting >>> turtlebot2_drivers
Starting >>> cartographer
Finished <<< turtlebot2_drivers Æ0.75sÅ                                                                    
Starting >>> turtlebot2_follower
Starting >>> turtlebot2_teleop
Starting >>> turtlebot2_amcl
Finished <<< turtlebot2_teleop Æ0.36sÅ                                                                                                    
Finished <<< turtlebot2_amcl Æ0.40sÅ
Finished <<< turtlebot2_follower Æ0.50sÅ                                                                    
Finished <<< cartographer Æ2.86sÅ                       
Starting >>> cartographer_ros
ÆProcessing: cartographer_rosÅ                             
Finished <<< cartographer_ros Æ37.8sÅ                         
Starting >>> turtlebot2_cartographer
Finished <<< turtlebot2_cartographer Æ2.85sÅ                   
                      
Summary: 7 packages finished Æ43.9sÅ
docker@f9c421174eaf:~/ros2_ws$ source /opt/ros/bouncy/setup.bash 
...
docker@f9c421174eaf:~/ros2_ws$ colcon build --symlink-install --packages-skip cartographer cartographer_ros cv_bridge opencv_tests ros1_bridge turtlebot2_amcl turtlebot2_drivers turtlebot2_follower turtlebot2_cartographer turtlebot2_teleop vision_opencv
...
--- stderr: astra_camera                                  
makeÆ3Å: warning: jobserver unavailable: using -j1.  Add '+' to parent make rule.
ThirdParty/PSCommon/BuildSystem/CommonDefs.mak:40: HOST_PLATFORM is x64
../../BuildSystem/CommonDefs.mak:40: HOST_PLATFORM is x64
../../ThirdParty/PSCommon/BuildSystem/CommonDefs.mak:40: HOST_PLATFORM is x64
../../ThirdParty/PSCommon/BuildSystem/CommonDefs.mak:40: HOST_PLATFORM is x64
../../../ThirdParty/PSCommon/BuildSystem/CommonDefs.mak:40: HOST_PLATFORM is x64
../../ThirdParty/PSCommon/BuildSystem/CommonDefs.mak:40: HOST_PLATFORM is x64
../../ThirdParty/PSCommon/BuildSystem/CommonDefs.mak:40: HOST_PLATFORM is x64
../../ThirdParty/PSCommon/BuildSystem/CommonDefs.mak:40: HOST_PLATFORM is x64
../../ThirdParty/PSCommon/BuildSystem/CommonDefs.mak:40: HOST_PLATFORM is x64
../../ThirdParty/PSCommon/BuildSystem/CommonDefs.mak:40: HOST_PLATFORM is x64
ThirdParty/PSCommon/BuildSystem/CommonDefs.mak:40: HOST_PLATFORM is x64
../../BuildSystem/CommonDefs.mak:40: HOST_PLATFORM is x64
../../ThirdParty/PSCommon/BuildSystem/CommonDefs.mak:40: HOST_PLATFORM is x64
../../ThirdParty/PSCommon/BuildSystem/CommonDefs.mak:40: HOST_PLATFORM is x64
../../../ThirdParty/PSCommon/BuildSystem/CommonDefs.mak:40: HOST_PLATFORM is x64
../../ThirdParty/PSCommon/BuildSystem/CommonDefs.mak:40: HOST_PLATFORM is x64
../../ThirdParty/PSCommon/BuildSystem/CommonDefs.mak:40: HOST_PLATFORM is x64
../../ThirdParty/PSCommon/BuildSystem/CommonDefs.mak:40: HOST_PLATFORM is x64
../../ThirdParty/PSCommon/BuildSystem/CommonDefs.mak:40: HOST_PLATFORM is x64
../../ThirdParty/PSCommon/BuildSystem/CommonDefs.mak:40: HOST_PLATFORM is x64
---
...
Summary: 12 packages finished Æ9.97sÅ
  1 package had stderr output: astra_camera
```

So I basically took the above commands and made it into a single `RUN` command
in the Dockerfile, but that isn't working out so well.

```
RUN cd ~/ros2_ws && \
    sudo rosdep init && \
    rosdep update && \
    echo "Rosdep installing" && \
    rosdep install --from-paths src \
                   --ignore-src \
                   --rosdistro $CHOOSE_ROS_DISTRO \
                   -y -r \
                   --skip-keys "console_bridge fastcdr fastrtps libopensplice67 rti-connext-dds-5.3.1 urdfdom_headers" &&
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
    rosdep init && \
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
      vision_opencv turtlebot2_demo
```

Basically, the same problelm that I was having while running commands inside the docker container itself happens if the commands are run from the Dockerfile itself.
