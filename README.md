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
###  0.) Start from the latest ROS 2 Humble system, build a "ros-humble-ros1-bridge/" ROS2 package:

``` bash
    cd ~/
    apt update; apt upgrade
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


## Troubleshoot

``` bash
$ ros2 run ros1_bridge dynamic_bridge --print-pairs | grep -i srv
```
```
  - 'diagnostic_msgs/srv/AddDiagnostics' (ROS 2) <=> 'diagnostic_msgs/AddDiagnostics' (ROS 1)
  - 'diagnostic_msgs/srv/SelfTest' (ROS 2) <=> 'diagnostic_msgs/SelfTest' (ROS 1)
  - 'example_interfaces/srv/AddTwoInts' (ROS 2) <=> 'roscpp_tutorials/TwoInts' (ROS 1)
  - 'example_interfaces/srv/AddTwoInts' (ROS 2) <=> 'rospy_tutorials/AddTwoInts' (ROS 1)
  - 'nav_msgs/srv/GetMap' (ROS 2) <=> 'nav_msgs/GetMap' (ROS 1)
  - 'nav_msgs/srv/GetPlan' (ROS 2) <=> 'nav_msgs/GetPlan' (ROS 1)
  - 'nav_msgs/srv/LoadMap' (ROS 2) <=> 'nav_msgs/LoadMap' (ROS 1)
  - 'nav_msgs/srv/SetMap' (ROS 2) <=> 'nav_msgs/SetMap' (ROS 1)
  - 'sensor_msgs/srv/SetCameraInfo' (ROS 2) <=> 'sensor_msgs/SetCameraInfo' (ROS 1)
  - 'std_srvs/srv/Empty' (ROS 2) <=> 'std_srvs/Empty' (ROS 1)
  - 'std_srvs/srv/SetBool' (ROS 2) <=> 'std_srvs/SetBool' (ROS 1)
  - 'std_srvs/srv/Trigger' (ROS 2) <=> 'std_srvs/Trigger' (ROS 1)
  - 'tf2_msgs/srv/FrameGraph' (ROS 2) <=> 'tf2_msgs/FrameGraph' (ROS 1)
```

``` bash
$ ros2 run ros1_bridge dynamic_bridge --print-pairs | grep -i tf2
```
```
  - 'tf2_msgs/msg/TF2Error' (ROS 2) <=> 'tf2_msgs/TF2Error' (ROS 1)
  - 'tf2_msgs/msg/TFMessage' (ROS 2) <=> 'tf2_msgs/TFMessage' (ROS 1)
  - 'tf2_msgs/msg/TFMessage' (ROS 2) <=> 'tf/tfMessage' (ROS 1)
  - 'tf2_msgs/srv/FrameGraph' (ROS 2) <=> 'tf2_msgs/FrameGraph' (ROS 1)
```



## References
- https://github.com/ros2/ros1_bridge
- https://github.com/mjforan/ros-humble-ros1-bridge
- https://github.com/ros2/ros1_bridge/issues/391
- https://packages.ubuntu.com/jammy/ros-robot-dev
