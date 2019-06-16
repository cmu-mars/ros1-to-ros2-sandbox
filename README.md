# ros1-to-ros2-sandbox

## File Organization

The purposes of the various folders and files:

- `dockerfiles/`: various dockerfiles for different builds
- `ex1/`: files pertaining to the example ROS1 vs. ROS2 code from the PDF, including launch files
  - note: as a test, this is referred to as test1 since it is example 1
- `launch/`: other launch files, for trying to run the turtlebot2_demo
- `rviz-diff/`: script for comparing ROS1 vs. ROS2 versions of packages
  - `src/`: outputted diffs for files judged to be the same, specifically for the rviz package
- `.dockerignore`: patterns that match files and directories that we don't want to be copied/added to a built image
- `my-bash*`: various basic `.bashrc` and `.bash_profile` files for sourcing different kinds of installations of ROS.
- `ros1-to-ros2-pack-spec.json`: a specification of how ROS core packages' names and organization changed
- `sudoers.txt`: file that specifies some permissions information for the user you enter a docker image as, called "docker."


## Docker Configuration

This has been tested on Mac OS X 10.12.6 running Docker Desktop 2.0.0.3, with Docker Engine version 18.09.2.


## Build

### Dockerfiles

All Dockerfiles are now located in `dockerfiles/`. Here are the important ones:

- `Dockerfile-bin-melodic`: comes with full ROS1 melodic installation, installs ROS2 Bouncy and turtlebot2_demo from binaries
- `Dockerfile-ros1-base`: full desktop melodic installation, also installs ROS developer and package creation/build tools
- `Dockerfile-ros2-base`: bouncy desktop install, and ROS developer/build dependencies
- `Dockerfile-vnc-melodic`: same as `Dockerfile-bin-melodic`, but enables X forwarding
- `test1-ros1`: takes the image you built from `Dockerfile-ros1` (assumes you've called it `<username>/ros1-base`), and creates a package called test1 that will do the ROS1 version of `Dockerfile-x-dev-melodic`
  - when you run this image, it automatically launches the test1 package
- `test1-ros2`: creates image based on the image built by `Dockerfile-ros2-base`, assumes you've called it `<username>/ros2-base`
  - creates and builds package, test launches immediately when the image is run
  
### Building a Dockerfile

```
docker build -t <username>/<image-name>:<version> -f dockerfiles/<dockerfile-name> .
```

For the `test1-ros1` and `test1-ros2` Dockerfiles, an extra command line
argument of `--build-arg USERNAME=<insert-username-here>` is needed.
This ensures that it references the correct image built from `Dockerfile-ros1-base`
or `Dockerfile-ros2-base`, respectively.

#### Image Naming and Versioning

For the most part, the names are arbitrary, though it is recommended 
that the image name at least in part reflects the Dockerfile it came from. There
are only two instances in which the naming isn't arbitrary, which were detailed in
[the dockerfile section](#dockerfiles). For the most part, the built image
should have the same name as the Dockerfile file name, minus the `Dockerfile` part.

The version is entirely optional. I just include it so that if I have a working docker
image and I'm making potentially breaking changes, I can just increase the version number
so it won't affect the currently "working" docker image. Be warned that this can eat up
the storage available to the Docker.

## Run

If you need to run the resulting image in one terminal, you can use

```
docker run -t -i <username>/<name-of-image>:<version>
```

The naming of the image needs to be made more consistent later, as
mentioned earlier.

If multiple terminals are needed, as is the case especially for
running something using the [ROS1 bridge](https://github.com/ros2/turtlebot2_demo#run-the-bridge) or 
any other use case involving a server, first start the image in
the background with

```
$ docker run -d -it <username>/ros2turtle:<version>
<container id>
$
```

This should output a `<container id>` as in the above, but if it doesn't,
then you can run `docker ps` to find the container id. Then in each new
terminal window, you should be able to do the following.

```
$ docker exec -it <container id> bash
docker@<container id>:~/ros2_ws$ 
```

which shows you an interactive terminal to the container.

When you enter the container, your working directory is automatically
the root directory.

### Exiting the Image

No matter how you ran the image, you should be able to exit it with

```
docker@<container id>$ exit
$
```

It may be necessary to kill the container, since it is running in the background.
If `docker ps` shows that the container `<container id>` is still running, use
`docker kill <container id>` which will make it exit with status 137.


## Known Issues

It has been difficult to set up X forwarding on Mac so that it communicates with
the Docker container. Numerous ways of doing this have been attempted, and many
tutorials appear to be old.

The system installed is also a little bit different because we instantiate a user,
called `docker`, that you enter the image as, instead of being `root` like many
other Docker containers.

