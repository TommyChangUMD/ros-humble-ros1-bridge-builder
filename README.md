# ros-humble-ros1-bridge-builder
Create a "*ros-humble-ros1-bridge*" package that can be used directly within Ubuntu 22.02 ROS2 Humble. Both amd64 and arm64 architectures are supported.

- Note1: It takes approximately 10 minutes on my PC, equipped with a 6-core CPU (12 logical cores) and 24GB of memory.

- Note2: It takes about 1 GB of memory per logical CPU core to compile the ROS1 bridge. So, if your system has only 4 GB of memory but 100 logical CPU cores, it will still use only 4 logical cores for the compilation. Now, why does it take so much memory to compile?  Well, you can blame the overuse of C++ templates...

## How to create this builder docker images:

``` bash
  git clone https://github.com/TommyChangUMD/ros-humble-ros1-bridge-builder.git
  cd ros-humble-ros1-bridge-builder

  # By default, ros-tutorals support will be built: (bridging the ros-humble-example-interfaces package)
  docker build . -t ros-humble-ros1-bridge-builder
```

- Note1: Since building a docker image just needs docker, you could do this step on any system that has docker installed -- it doesn't have to on a Ubuntu 22.04 and it doesn't need ROS2 neither.

- Note2: The builder image is multi-platform and can be created on an amd64 machine (e.g., Intel and AMD CPUs) or an arm64 machine (e.g., Raspberry Pi 4B and Nvidia Jetson Orin).

Alternative builds:
``` bash
  # **[OPTIONAL]** If you don't want to build ros-tutorals support:
  docker build . --build-arg ADD_ros_tutorials=0 -t ros-humble-ros1-bridge-builder

  # **[OPTIONAL]** If you want to build grid-map support:  (bridging the ros-humble-grid-map package)
  docker build . --build-arg ADD_grid_map=1 -t ros-humble-ros1-bridge-builder

  # **[OPTIONAL]** If you want to build an example custom message:
  docker build . --build-arg ADD_custom_msgs=1 -t ros-humble-ros1-bridge-builder
```
- Note1: Don't forget to install the necessary `ros-humble-grid-map` packages on your ROS2 Humble if you choose to build the bridge with the `grid-map` support added.

- Note2: For the custom message example, there is no pre-build package for ROS2 Humble so you will need to compile it from the source.  For details, see [Checking example custom message](#checking-example-custom-message) in the Troubleshoot section.

## How to create ros-humble-ros1-bridge package:
###  0.) Start from the latest Ubuntu 22.04 ROS 2 Humble Desktop system, create the "ros-humble-ros1-bridge/" ROS2 package:

``` bash
    cd ~/
    apt update; apt upgrade
    apt -y install ros-humble-desktop
    docker run --rm ros-humble-ros1-bridge-builder | tar xvzf -
```

- Note1: It's **important** that you have **`ros-humble-desktop`** installed on your ROS2 Humble system because we want to **match it with the builder image as closely as possible**.  So, if you haven't done so already, do:
``` bash
    apt -y install ros-humble-desktop
```
Otherwise you may get an error about missing `ibexample_interfaces__rosidl_typesupport_cpp.so`.  See issue https://github.com/TommyChangUMD/ros-humble-ros1-bridge-builder/issues/10


- Note1: There is no compilation at this point, the `docker run` command simply spits out a pre-compiled tarball for either amd64 or arm64 architecture, depending on the architecture of the machine you used to created the builder image.

- Note2: The assumption is that this tarball contains configurations and libraries matching your ROS2 Humble system very closely, although not identical.

- Note3: We don't really need the builder image anymore, to delete it, do:

``` bash
    docker rmi ros-humble-ros1-bridge-builder
```

## How to use ros-humble-ros1-bridge:
###  1.) First start a ROS1 Noetic docker and bring up a GUI terminal, something like:

``` bash
  rocker --x11 --user --privileged \
         --volume /dev/shm /dev/shm --network=host -- ros:noetic-ros-base-focal \
         'bash -c "sudo apt update; sudo apt install -y ros-noetic-rospy-tutorials tilix; tilix"'
```

Tha docker image used above `ros:noetic-ros-base-focal` is multi-platform.  It runs on amd64 (eg., Intel and AMD cups) or arm64 architecture (eg., Raspberry PI 4B and Nvidia Jetson Orin).

You may need to install rocker first:
``` bash
  sudo apt install python3-rocker
```
- Note1: It's important to share the host's network and the `/dev/shm/` directory with the container.
- Note2: You can add the `--home` rocker option if you want your home directory to be shared with the docker container.  Be careful though, as the host's `~/.bashrc` will be executed inside the container.
- Note3: You can also use **ROS1 Melodic**.  Just replace `ros:noetic-ros-base-focal` with `ros:melodic-ros-base-bionic` and also replace `ros-noetic-rospy-tutorials` with `ros-melodic-rospy-tutorials`.

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
  # or try:
  ros2 run ros1_bridge dynamic_bridge --bridge-all-topics
```
- Note: We need to source `local_setup.bash` and NOT `setup.bash` because the bridge was compiled in a docker container that may have different underlay locations.  Besides, we don't need to source these underlays in the host system again.

###  3.) Back to the ROS1 Noetic docker container, run in another terminal tab:

``` bash
  source /opt/ros/noetic/setup.bash
  rosrun rospy_tutorials talker
```

###  4.) Finally, from the Ubuntu 22.04 ROS2 Humble system, run in another terminal tab:

``` bash
  source /opt/ros/humble/setup.bash
  ros2 run demo_nodes_cpp listener
```

## How to add custom message from ROS1 and ROS2 source code
See an step 6.3 and 7 in the Dockerfile for an example.

- Note1: Make sure the package name ends with "_msgs".
- Note2: Use the same package name for both ROS1 and ROS2.

Also see the [troubleshoot section](#checking-example-custom-message).

- ref: https://github.com/TommyChangUMD/custom_msgs.git
- ref: https://github.com/ros2/ros1_bridge/blob/master/doc/index.rst


## How to make it work with ROS1 master running on a different machine?
- Run `roscore` on the Noetic machine as usual.
- On the Humble machine, run the bridge as below (assuming the IP address of the Noetic machine is 192.168.1.208):

``` bash
  source /opt/ros/humble/setup.bash
  source ~/ros-humble-ros1-bridge/install/local_setup.bash
  ROS_MASTER_URI='http://192.168.1.208:11311' ros2 run ros1_bridge dynamic_bridge
  # Note, change "192.168.1.208" above to the IP address of your Noetic machine.
```

## Troubleshoot

### Fixing "[ERROR] Failed to contact master":

If you have Noetic and Humble running on two different machines and have
already set the ROS_MASTER_URI environment variable, you should check the
network to ensure that the Humble machine can reach the Noetic machine via
port 11311.

``` bash
$ nc -v -z 192.168.1.208 11311
# Connection to 192.168.1.208 11311 port [tcp/*] succeeded!
```

### Checking tf2 message / service:
``` bash
$ ros2 run ros1_bridge dynamic_bridge --print-pairs | grep -i tf2
  - 'tf2_msgs/msg/TF2Error' (ROS 2) <=> 'tf2_msgs/TF2Error' (ROS 1)
  - 'tf2_msgs/msg/TFMessage' (ROS 2) <=> 'tf2_msgs/TFMessage' (ROS 1)
  - 'tf2_msgs/msg/TFMessage' (ROS 2) <=> 'tf/tfMessage' (ROS 1)
  - 'tf2_msgs/srv/FrameGraph' (ROS 2) <=> 'tf2_msgs/FrameGraph' (ROS 1)
```

### Checking AddTwoInts message / service:
- By default, `--build-arg ADD_ros_tutorials=1` is implicitly added to the `docker build ...` command.
- The ROS2 Humble system must have the `ros-humble-example-interfaces` package installed.
``` bash
$ sudo apt -y install ros-humble-example-interfaces
$ ros2 run ros1_bridge dynamic_bridge --print-pairs | grep -i addtwoints
  - 'example_interfaces/srv/AddTwoInts' (ROS 2) <=> 'roscpp_tutorials/TwoInts' (ROS 1)
  - 'example_interfaces/srv/AddTwoInts' (ROS 2) <=> 'rospy_tutorials/AddTwoInts' (ROS 1)
```

### Checking grid-map message / service:
- Must have `--build-arg ADD_grid_map=1` added to the `docker build ...` command.
- Note: In addition, the ROS2 Humble system must have the `ros-humble-grid-map` package installed.
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

### Checking example custom message:
- Thanks to [Codaero](https://github.com/Codaero) for the custom message source code.
- Must have `--build-arg ADD_custom_msgs=1` added to the `docker build ...` command.
``` bash
# First, install the ROS2 pacakge from the source
$ git clone https://github.com/TommyChangUMD/custom_msgs.git
$ cd custom_msgs/custom_msgs_ros2
$ source /opt/ros/humble/setup.bash
$ colcon build
$ source install/setup.bash

# Now, run the bridge
$ source ~/ros-humble-ros1-bridge/install/local_setup.bash
$ ros2 run ros1_bridge dynamic_bridge --print-pairs | grep -i PseudoGridMap
  - 'custom_msgs/msg/PseudoGridMap' (ROS 2) <=> 'custom_msgs/PseudoGridMap' (ROS 1)
```


## References
- https://github.com/ros2/ros1_bridge
- https://github.com/ros2/ros1_bridge/blob/master/doc/index.rst
- https://github.com/smith-doug/ros1_bridge/tree/action_bridge_humble
- https://github.com/mjforan/ros-humble-ros1-bridge
- https://packages.ubuntu.com/jammy/ros-desktop-dev
