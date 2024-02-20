# ros-humble-ros1-bridge-builder
Create a "*ros-humble-ros1-bridge*" package that can be used directly within Ubuntu 22.02 ROS2 Humble.

It takes approximately 10 minutes on my PC, equipped with a 6-core CPU and 24GB of memory.

## How to create this builder docker image:

``` bash
  git clone https://github.com/TommyChangUMD/ros-humble-ros1-bridge-builder.git
  cd ros-humble-ros1-bridge-builder

  # By default, ros-tutorals support will be built: (bridging the ros-humble-example-interfaces package)
  docker build . -t ros-humble-ros1-bridge-builder
```

Alternative builds:
```
  # **[OPTIONAL]** If you don't want to build ros-tutorals support:
  docker build . --build-arg ADD_ros_tutorials=0 -t ros-humble-ros1-bridge-builder
  
  # **[OPTIONAL]** If you want to build grid-map support:  (bridging the ros-humble-grid-map package)
  docker build . --build-arg ADD_grid_map=1 -t ros-humble-ros1-bridge-builder
```

*Note: Since building the docker image just needs docker, you could do this step on any system that has docker installed -- it doesn't have to on a Ubuntu 22.04 and it doesn't need ROS2 neither.

## How to create ros-humble-ros1-bridge package:
###  0.) Start from the latest Ubuntu 22.04 ROS 2 Humble Desktop system, create the "ros-humble-ros1-bridge/" ROS2 package:

``` bash
    cd ~/
    apt update; apt upgrade
    apt -y install ros-humble-desktop
    docker run --rm ros-humble-ros1-bridge-builder | tar xvzf -
```

Note1, it's important that you have `ros-humble-desktop` installed on your ROS2 Humble system because we want to **match the builder image as closely as possible**.

Note1: There is no compilation at this point, the `docker run` command simply spits out a pre-compiled tarball.  The assumption is that this tarball contains configurations and libraries matching your ROS2 Humble system very closely, although not identical.

Note2: We don't need the builder image anymore, to delete it, do:

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

###  3.) Now, from the Ubuntu 22.04 ROS2 Humble system, start the ros1 bridge node.

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

###  4.) Finally, from the Ubuntu 22.04 ROS2 Humble system:

``` bash
  source /opt/ros/humble/setup.bash
  ros2 run demo_nodes_cpp listener
```


## Troubleshoot

### Check tf2 message / service
``` bash
$ ros2 run ros1_bridge dynamic_bridge --print-pairs | grep -i tf2
  - 'tf2_msgs/msg/TF2Error' (ROS 2) <=> 'tf2_msgs/TF2Error' (ROS 1)
  - 'tf2_msgs/msg/TFMessage' (ROS 2) <=> 'tf2_msgs/TFMessage' (ROS 1)
  - 'tf2_msgs/msg/TFMessage' (ROS 2) <=> 'tf/tfMessage' (ROS 1)
  - 'tf2_msgs/srv/FrameGraph' (ROS 2) <=> 'tf2_msgs/FrameGraph' (ROS 1)
```

### Check AddTwoInts message / service
By default, `--build-arg ADD_ros_tutorials=1` is implicitly added to the `docker build ...` command.

Note: In addition, the ROS2 Humble system must have the `ros-humble-example-interfaces` package installed.
``` bash
$ sudo apt -y install ros-humble-example-interfaces
$ ros2 run ros1_bridge dynamic_bridge --print-pairs | grep -i addtwoints
  - 'example_interfaces/srv/AddTwoInts' (ROS 2) <=> 'roscpp_tutorials/TwoInts' (ROS 1)
  - 'example_interfaces/srv/AddTwoInts' (ROS 2) <=> 'rospy_tutorials/AddTwoInts' (ROS 1)
```

### Check grid-map message / service
Must have `--build-arg ADD_grid_map=1` added to the `docker build ...` command.

Note: In addition, the ROS2 Humble system must have the `ros-humble-grid-map` package installed.
``` bash
$ sudo apt -y install ros-humble-grid-map
$ ros2 run ros1_bridge dynamic_bridge --print-pairs | grep -i grid_map
  - 'grid_map_msgs/msg/GridMap' (ROS 2) <=> 'grid_map_msgs/GridMap' (ROS 1)
  - 'grid_map_msgs/msg/GridMapInfo' (ROS 2) <=> 'grid_map_msgs/GridMapInfo' (ROS 1)
  - 'grid_map_msgs/srv/GetGridMap' (ROS 2) <=> 'grid_map_msgs/GetGridMap' (ROS 1)
  - 'grid_map_msgs/srv/GetGridMapInfo' (ROS 2) <=> 'grid_map_msgs/GetGridMapInfo' (ROS 1)
  - 'grid_map_msgs/srv/ProcessFile' (ROS 2) <=> 'grid_map_msgs/ProcessFile' (ROS 1)
  - 'grid_map_msgs/srv/SetGridMap' (ROS 2) <=> 'grid_map_msgs/SetGridMap' (ROS 1)
```

#### Error: gibgrid_map_msgs__rosidl_typesupport_cpp.so: cannot open shared object file: No such file or directory
``` bash
$ sudo apt -y install ros-humble-grid-map
```

## References
- https://github.com/ros2/ros1_bridge
- https://github.com/smith-doug/ros1_bridge/tree/action_bridge_humble
- https://github.com/mjforan/ros-humble-ros1-bridge
- https://packages.ubuntu.com/jammy/ros-desktop-dev
