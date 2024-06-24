# iRC ROS Examples

This package showcases how to implement Igus Robot Control ROS2 for different usecases.

## Keyboard Control

This node can be used to control a mobile platform with Rebel arm via keyboard. Its main purpose is to show how to access all necessary State- and CommandInterfaces in order to control the platform via the CRI-Interface ([see also cpr-platform controller](https://github.com/CommonplaceRobotics/iRC_ROS/blob/RebelCarrier/irc_ros_controllers/src/platform_controller.cpp)). To run this example first launch irc_ros_bringup/launch/rebel_on_platform.launch.py
```
ros2 launch irc_ros_bringup rebel_on_platform.launch.py hardware_protocol:=cri use_rviz:=false
```
You can now launch the keyboard control node in a second terminal:

```
ros2 run irc_ros_examples keyboard_ctrl
```
Once the node is running you can control the platform using these keys: 

| Key   | Function | 
| :---: | :------: | 
| r     |   reset   | 
| e     |   enable motors  | 
| w     |  disable motors  | 
| m     | toggle motiontype |


|axis (joint motion) |axis (cartesian motion) | Key(+) | Key(-)|
|:----:              |            :---:|       :---:|   :---: |
| 1                  | X Translation          | a |   y |
| 2                  |Y Translation           | s |   x |
| 3                  |z-translation           | d |    c|
| 4                  |x-rotation              | f |    v|
| 5                  | y-rotation             | g |    b|
| 6                  | z-rotation             | h |    n|
