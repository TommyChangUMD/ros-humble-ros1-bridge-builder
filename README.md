# ros-humble-ros1-bridge-builder
Create the "*ros-humble-ros1-bridge*" package that can be used
directly in ROS2 Humble.  

It takes only 10 minutes.


## How to build this docker image:

``` bash
  docker build . -t ros-humble-ros1-bridge-builder
```

## How to build ros1_humble_bridge:
###  0.) Start from the ROS 2 Humble system, build a "ros1_humble_bridge/" ROS2 package:

``` bash
  docker run --rm ros-humble-ros1-bridge-builder | tar xvzf -
```

## How to use ros1_humble_bridge:
###  1.) First start a ROS1 Noetic docker and bring up a GUI terminal, something like:

``` bash
  rocker --x11 --user --home --privileged \
         --volume /dev/shm /dev/shm --network=host -- osrf/ros:noetic-desktop \
         'bash -c "sudo apt update; sudo apt install -y tilix; tilix"'
```

###  2.) Then, start "roscore" inside the ROS1 docker

``` bash
  source /opt/ros/noetic/setup.bash
  roscore
```

###  3.) Now, from the ROS2 Humble system, start the ros1 bridge node.

``` bash
  source /opt/ros/humble/setup.bash
  source ros1_humble_bridge/install/setup.bash
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

