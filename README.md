# ros-humble-ros1-bridge-builder
Create the "*ros-humble-ros1-bridge*" package that can be used
directly in ROS2 Humble.  

It takes only 10 minutes.


## How to create this builder docker image:

``` bash
  docker build . -t ros-humble-ros1-bridge-builder
```

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

###  2.) Then, start "roscore" inside the ROS1 Noetic docker container

``` bash
  source /opt/ros/noetic/setup.bash
  roscore
```

###  3.) Now, from the ROS2 Humble system, start the ros1 bridge node.

``` bash
  source /opt/ros/humble/setup.bash
  source ~/ros-humble-ros1-bridge/install/setup.bash
  ros2 run ros1_bridge dynamic_bridge
```

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
