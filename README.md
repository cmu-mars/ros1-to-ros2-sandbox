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

In the last commit, I figured out that `rosdep` was having problems
with finding the dependencies for opencv3, which is probably why `colcon
build` was very unhappy. (If you want a full overview of this, just look at
the readme that was in place for the last commit.)

A quick internet search found that apparently this is because the ros package
is not available as a binary, so you need to build it from source,
according to [this webpage](https://answers.ros.org/question/229300/knowrob_cad_models-cannot-locate-rosdep-definition-for-iai_cad_downloader/).

Now, we did have the flag `--from-paths src` that allows it, I think, to actually
build it from source, but the full recommended install line was

```
rosdep install --from-paths src --ignore-src --rosdistro kinetic -y -r
```

(where we would replace `kinetic` with `bouncy` or something) and the `-r` was
the part that we were missing. So, the new command to run would be

```
rosdep install --from-paths src --ignore-src --rosdistro bouncy -y -r --skip-keys "console_bridge fastcdr fastrtps libopensplice67 rti-connext-dds-5.3.1 urdfdom_headers"
```

I've updated the Dockerfile with these insights in mind.

```
#RUN cd ~/ros2_ws && \
#    sudo rm /etc/ros/rosdep/sources.list.d/20-default.list && \
#    sudo rosdep init && \
#    rosdep update && \
#    rosdep install --from-paths src --ignore-src --rosdistro bouncy -y -r --skip-keys "console_bridge fastcdr fastrtps libopensplice67 rti-connext-dds-5.3.1 urdfdom_headers"
#    . /opt/ros/bouncy/setup.sh && \
#    colcon build --symlink-install --packages-skip cartographer cartographer_ros cv_bridge opencv_tests ros1_bridge turtlebot2_amcl turtlebot2_drivers turtlebot2_follower turtlebot2_cartographer turtlebot2_teleop vision_opencv
```

When I ran the above lines in the Docker image itself,
which essentially builds everything up to the point above,
this is what I got:

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
docker@f9c421174eaf:~/ros2_ws$ rosdep install --from-paths src --ignore-src --rosdistro bouncy -y -r --skip-keys "console_bridge fastcdr fastrtps libopensplice67 rti-connext-dds-5.3.1 urdfdom_headers"
ERROR: the following packages/stacks could not have their rosdep keys resolved
to system dependencies:
cv_bridge: Cannot locate rosdep definition for Æopencv3Å
Continuing to install resolvable dependencies...
executing command Æsudo -H apt-get install -y libprotoc-devÅ
Reading package lists... Done
...
#All required rosdeps installed successfully
docker@f9c421174eaf:~/ros2_ws$ source /opt/ros/bouncy/setup.bash
ROS_DISTRO was set to 'melodic' before. Please make sure that the environment does not mix paths from different distributions.
docker@f9c421174eaf:~/ros2_ws$ colcon build --symlink-install --packages-skip cartographer cartographer_ros cv_bridge opencv_tests ros1_bridge turtlebot2_amcl turtlebot2_drivers turtlebot2_follower turtlebot2_cartographer turtlebot2_teleop vision_opencv
Æ0.361sÅ WARNING:colcon.colcon_core.package_selection:ignoring unknown package 'ros1_bridge' in --packages-skip
Starting >>> image_geometry
Starting >>> map_server
Starting >>> astra_camera
Starting >>> cartographer_ros_msgs
Finished <<< image_geometry Æ0.59sÅ                                                                                                      
Starting >>> joy
Finished <<< map_server Æ0.70sÅ                                                                       
Starting >>> teleop_twist_joy
Finished <<< joy Æ0.38sÅ                                                                                                               
Starting >>> amcl
Finished <<< teleop_twist_joy Æ0.42sÅ                                                                            
Starting >>> depthimage_to_laserscan
Finished <<< amcl Æ0.68sÅ                                                                                                                    
Starting >>> depthimage_to_pointcloud2
Finished <<< cartographer_ros_msgs Æ1.70sÅ                                                                        
Starting >>> pcl_conversions
Finished <<< depthimage_to_laserscan Æ0.71sÅ                                                                        
Starting >>> teleop_twist_keyboard
Finished <<< depthimage_to_pointcloud2 Æ0.70sÅ                                                                        
Finished <<< pcl_conversions Æ0.75sÅ                                                                                                  
Finished <<< teleop_twist_keyboard Æ0.79sÅ                                                       
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
---Æ6.714sÅ ERROR:colcon.colcon_cmake.task.cmake.build:Failed to find the following files:
- /home/docker/ros2_ws/install/cartographer/share/cartographer/package.sh
- /home/docker/ros2_ws/install/turtlebot2_drivers/share/turtlebot2_drivers/package.sh
- /home/docker/ros2_ws/install/cartographer_ros/share/cartographer_ros/package.sh
- /home/docker/ros2_ws/install/turtlebot2_follower/share/turtlebot2_follower/package.sh
- /home/docker/ros2_ws/install/turtlebot2_teleop/share/turtlebot2_teleop/package.sh
- /home/docker/ros2_ws/install/turtlebot2_amcl/share/turtlebot2_amcl/package.sh
- /home/docker/ros2_ws/install/turtlebot2_cartographer/share/turtlebot2_cartographer/package.sh
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
docker@f9c421174eaf:~/ros2_ws$ 
```

So this is way better than what I had before! Now it finishes 11/13 packages,
instead of just 0/13. However, this still isn't ideal? I'm also not sure
where it wants me to "add '+' to [the] parent make rule."

Note that the packages that it wants you to check (`cartographer`, `turtlebot2_drivers`, `cartographer_ros`, `turtlebot2_follower`, etc.) are all of the ones skipped in this step:

```
colcon build --symlink-install --packages-skip cartographer cartographer_ros cv_bridge opencv_tests ros1_bridge turtlebot2_amcl turtlebot2_drivers turtlebot2_follower turtlebot2_cartographer turtlebot2_teleop vision_opencv
```

In fact, these are built in the second part [described here in the instructions for building turtlebot2_demo](https://github.com/ros2/turtlebot2_demo#build-the-ros2-code). They were moved second because...they're more resource intensive and rely on ROS1 packages.

My next step would be to try doing that first,
but I'm going to bed because it's 1:00 am.
