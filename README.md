# ROS2 packages for igus EduMove

**This is an early beta version. The code WILL change in the near future and is not intended for any serious work right now. If you still use it be cautious when running it on your hardware and know that updating this package may break your running setup!**

This repository aims to bring ROS2 support for the igus EduMove. It is based on [iRC_ROS](https://github.com/CommonplaceRobotics/iRC_ROS). 

## Overview
The structure of the project tries to follow the general structure of similar ROS2 projects. The following graph tries to provide a general overview of the different packages and how they interact with each other:

![An overview of the different packages](doc/Architecture.png)

For basic robot interaction only irc_ros_hardware, irc_ros_description and irc_ros_bringup are required. The hardware package is used for the communication itself, description contains the robot definitions and bringup the necessary files to start the underlying software.
For robot motion planing moveit is used, the launch file for an igus ReBeL can be found in the irc_ros_moveit_config package. When using a mobile platform Navigation 2 is used, again with the files contained in the irc_ros_navigation package.
The controllers package is used to provide additional module functionality, one of them is providing the data for the dashboard package. For all this the interfacing uses custom messages, which can be found in the respective package.
Last, for some implementation examples please take a look at irc_ros_examples.

## Requirements
This project will only run under linux. One of the main issues is the use of linux sockets for interfacing with the hardware. If an adequate, open source and multi-platform alternative exists, it might be possible for the community to replace the linux-only parts.

**[Note: There appear to be issues using CycloneDDS and the digital IO modules together. If you run into problems please consider using a different implementation](https://github.com/CommonplaceRobotics/iRC_ROS/issues/105)**

General requirements:
 - Ubuntu 22.04 (Recommended)
 - Compiler with C++17 support, e.g. a semi-recent gcc install
 - [ROS2 Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)
   - Might work with ROS2 Iron out of the box as well, but is untested
 - Ethernet connection to the `TinyCtrl` host
 - [Navigation2](https://navigation.ros.org/build_instructions/index.html) (Most launch files use Nav2 functions for yaml processing)
 - Optional, only required for parts of the project:
   - [Moveit2 (stable)](https://moveit.ros.org/install-moveit2/binary/)
   - [Textual](https://github.com/Textualize/textual)

### Realtime kernel
While ros2_control recommends the use of a real-time kernel we also used a standard Ubuntu 22.04 install without encountering any issues. For installing the RT patch follow [this](https://docs.ros.org/en/humble/Tutorials/Miscellaneous/Building-Realtime-rt_preempt-kernel-for-ROS-2.html) guide.

When compiling the kernel for Ubuntu (22.04) make sure to also install the `dwarves` package before compiling. In the .config file remove the ubuntu certificate names and disable secure boot, else the missing certificates will prevent booting.

## Setup
Install all necessary requirements for the functionality you intend to use and simply clone this in the src/ directory of a ROS2 workspace. You can clone all requires ROS2 packages by running the following command in the workspace folder:

```
vcs import src --input src/EduMove-ROS2/EduMove-ROS2.repos
```

### CRI specific setup
When using CRI make sure the [Embedded Control](https://cpr-robots.com/robot-control#electronics) is reachable under the ip address set in the `irc_ros_description/urdf/*.ros2_control.xacro` configs (default: `192.168.3.11`) You should be able to ping the module in case you are unsure if the ip address is correct

## Contributing
CI is done via a Github Action calling a pre-commit file. Please test your pull requests before pushing by running run pre-commit locally.

## Links

## Contact and support
If you have any questions regarding the ROS2 package please post an issue on github. For any questions regarding the hardware please contact the Commonplace Robotics support at support@cpr-robots.com.

<p align="center">
  <img alt="Commonplace Robotics" src="./doc/logo_cpr.jpg" width="45%">
&nbsp; &nbsp; &nbsp; &nbsp;
  <img alt="igus Low Cost Automation" src="./doc/logo_igus_lca.jpg" width="45%">
</p>
