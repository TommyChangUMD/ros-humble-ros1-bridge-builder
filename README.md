# ros-humble-ros1-bridge-builder
Create a "*ros-humble-ros1-bridge*" package that can be used directly within Ubuntu 22.02 ROS2 Humble.

It takes approximately 10 minutes on my PC, equipped with a 6-core CPU and 24GB of memory.

## How to create this builder docker image:

``` bash
  git clone https://github.com/TommyChangUMD/ros-humble-ros1-bridge-builder.git
  cd ros-humble-ros1-bridge-builder
  docker build . -t ros-humble-ros1-bridge-builder
```

*Note: Since building the docker image just needs docker, you could do this step on any system that has docker installed -- it doesn't have to on a Ubuntu 22.04 and it doesn't need ROS2 neither.

## How to create ros-humble-ros1-bridge package:
###  0.) Start from the ROS 2 Humble system, build a "ros-humble-ros1-bridge/" ROS2 package:

``` bash
    cd ~/
    docker run --rm ros-humble-ros1-bridge-builder | tar xvzf -
```

We don't need the builder image anymore, to delete it, do:

``` bash
    docker rmi ros-humble-ros1-bridge-builder
```

## How to use ros-humble-ros1-bridge:
###  1.) First start a ROS1 Noetic docker and bring up a GUI terminal, something like:

``` bash
  rocker --x11 --user --home --privileged \
         --volume /dev/shm /dev/shm --network=host -- osrf/ros:noetic-desktop \
         'bash -c "sudo apt update; sudo apt install -y tilix; tilix"'
```
You may need to install rocker first:
``` bash
  sudo apt install python3-rocker
```
Note: It's important to share the host's network and the `/dev/shm/` directory with the container.

###  2.) Then, start "roscore" inside the ROS1 Noetic docker container

``` bash
  source /opt/ros/noetic/setup.bash
  roscore
```

###  3.) Now, from the ROS2 Humble system, start the ros1 bridge node.

``` bash
  source /opt/ros/humble/setup.bash
  source ~/ros-humble-ros1-bridge/install/local_setup.bash
  ros2 run ros1_bridge dynamic_bridge
```
*Note: We need to source `local_setup.bash` and NOT `setup.bash` because the bridge was compiled in a docker container that may have different underlay locations.  Besides, we don't need to source these underlays in the host system again.

###  3.) Back to the ROS1 Noetic docker container, run in another terminal tab:

``` bash
  source /opt/ros/noetic/setup.bash
  rosrun rospy_tutorials talker
```

###  4.) Finally, from the ROS2 Humble system:

``` bash
  source /opt/ros/humble/setup.bash
  ros2 run demo_nodes_cpp listener
```


## References
- https://github.com/ros2/ros1_bridge
- https://github.com/mjforan/ros-humble-ros1-bridge
- https://github.com/ros2/ros1_bridge/issues/391
- https://packages.ubuntu.com/jammy/ros-core-dev
