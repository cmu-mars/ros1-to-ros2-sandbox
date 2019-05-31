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
in it that do not build.

```
#RUN colcon build --symlink-install --packages-skip cartographer cartographer_ros cv_bridge opencv_tests ros1_bridge turtlebot2_amcl turtlebot2_drivers turtlebot2_follower turtlebot2_cartographer turtlebot2_teleop vision_opencv

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

There seems to be something wrong with `colcon build`, and the error that I get is

```
docker@736e301b590a:/$ colcon build --symlink-install --packages-skip cartographer cartographer_ros cv\_bridge opencv\_tests ros1\_bridge turtlebot2\_amcl turtlebot2\_drivers turtlebot2\_follower turtlebot2\_cartographer turtlebot2\_teleop vision\_opencv

Traceback (most recent call last):
  File "/usr/bin/colcon", line 11, in <module>
    load_entry_point('colcon-core==0.3.22', 'console_scripts', 'colcon')()
  File "/usr/lib/python3/dist-packages/colcon_core/command.py", line 121, in main
    create_log_path(args.verb_name)
  File "/usr/lib/python3/dist-packages/colcon_core/location.py", line 141, in create_log_path
    os.makedirs(str(path))
  File "/usr/lib/python3.6/os.py", line 210, in makedirs
    makedirs(head, mode, exist_ok)
  File "/usr/lib/python3.6/os.py", line 220, in makedirs
    mkdir(name, mode)
PermissionError: ÆErrno 13Å Permission denied: 'log'
```

I'm not sure what to make of this error.
