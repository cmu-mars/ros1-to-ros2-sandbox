# ros1-to-ros2-sandbox

## Docker Configuration

This has been tested on Mac OS X 10.12.6 running Docker Desktop 2.0.0.3, with Docker Engine version 18.09.2.

It's recommended, if possible, to allot at least 4 GiB of memory to Docker via the Docker preferences, otherwise certain parts of the build make take a very long time to run/fail out.

## Build

All Dockerfiles are now contained within the `dockerfiles` directory. The Dockerfile recommended for building is `dockerfiles/Dockerfile-bin-melodic`, which will install everything from binaries.

Build like so:

```
docker build -t <username>/ros2turtle:<version> -f dockerfiles/<dockerfile-name> .
```
Build is now much faster than it was previously.

Note: the name can be changed, and is a bit arbitrary, and the
only reason why I've been using a name is because then you can
separately run and build, so this is just what I've been doing to
get my hands on this in the first place. I'm not sure what Docker
best practices are, but this seems reasonable.

I also just include a version so that if I have a working docker
image and I'm making changes, I can just increase the version number
so it won't affect the currently "working" docker image.

## Run

If you need to run the resulting image in one terminal, you can use

```
docker run -t -i <username>/ros2turtle:<version>
```

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

I have been able to 
