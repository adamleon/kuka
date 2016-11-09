# kuka_ipk
Repository based on [kuka_experimental](https://github.com/ros-industrial/kuka_experimental). Holds support files, moveit configurations and network configurations specific to the IPK laboratory.

## Prerequisite
* A working ROS Environment. ROS Indigo can be installed using [this guide](http://wiki.ros.org/indigo/Installation/Ubuntu)
* ros_control `sudo apt-get install ros-indigo-ros-control ros-indigo-ros-controllers`
* MoveIt! `sudo apt-get install ros-indigo-moveit-full`

## Instructions
* Create and initialize a workspace, explained in [this ROS Tutorial](http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment)
* Navigate to `/your/workspace/src/`
* Run the following commands to install the packages:
```
git clone git@git.ipk.ivt.ntnu.no:adamleon/kuka_ipk.git
wstool init
wstool merge kuka_ipk/kuka.rosinstall
wstool update
cd ..
catkin_make
```

