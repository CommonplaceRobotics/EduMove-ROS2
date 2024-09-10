# iRC ROS Controllers

This package contains the custom controllers for the iRC ROS project:
- [DIO Controller](#dio-controller)


## DIO Controller
Adds a controller for using DIOs via CAN. The command outputs are for the Rebel_01, if you have a very early rebel model or a different robot please note that it will look different for you. The ReBeL 01 has two inputs and outputs in the arm and seven of each in the base.

### ROS2 Control specific commands
When successfully using this package you should be able to see the DIO Controller:

``` console
$ ros2 control list_controllers

dio_controller      [irc_ros_controllers/DIOController] active    
[...]

$ ros2 control list_hardware_interfaces

command interfaces
	dio_arm/digital_output_0 [available] [claimed]
	dio_arm/digital_output_1 [available] [claimed]
	dio_base/digital_output_0 [available] [claimed]
	dio_base/digital_output_1 [available] [claimed]
	dio_base/digital_output_2 [available] [claimed]
	dio_base/digital_output_3 [available] [claimed]
	dio_base/digital_output_4 [available] [claimed]
	dio_base/digital_output_5 [available] [claimed]
	dio_base/digital_output_6 [available] [claimed]
    [...]

state interfaces
	dio_arm/digital_input_0
	dio_arm/digital_input_1
	dio_base/digital_input_0
	dio_base/digital_input_1
	dio_base/digital_input_2
	dio_base/digital_input_3
	dio_base/digital_input_4
	dio_base/digital_input_5
	dio_base/digital_input_6
    [...]

```

Note that the hardware command interfaces are claimed by this controller and are provided by the `irc_ros_hardware` package, more precise by the CAN implementation.

### Setting states
The setting of states is done over messages or service calls.
To see all available output controllers you can use:
``` console
$ ros2 topic list
[...]
/dio_controller/set_outputs
[...]
/external_dio_controller/set_outputs
[...]

$ ros2 service list
[...]
/dio_controller/set_outputs
[...]
/external_dio_controller/set_outputs
[...]
```

Currently you can only see which outputs are available via the controller configuration file. In the future a `list_outputs` command could be an option.

Setting states works like the following:

``` console 
$ ros2 topic pub --once /dio_controller/set_outputs irc_ros_msgs/msg/DioCommand '{names:[dio_arm/digital_output_0, ],outputs: [False, ]}'

$ ros2 service call /dio_controller/set_outputs irc_ros_msgs/srv/DioCommand '{names:[dio_arm/digital_output_0, ],outputs: [False, ]}'
```

The service call will respond with success unless the number of outputs names and states do not match.

### Reading out states
Currently there is no way to read out all inputs via one topic. Reading out a single digital input works over the corrosponding `/[module_name]/[input_name]/state` topic, e.g.:

The available controllers can be seen with the following command:
``` console
$ ros2 topic list
[...]
/dio_controller/get_inputs
[...]
/external_dio_controller/get_inputs
[...]
```

## Platform Controller
Example Controller for a mobile Platform carrying a 6 axis ReBel-Cobot, that shows how to use the command-interfaces defined in the irc_ros_hardware package. 

## See:
 - [ROS2_Control: Writing a new controller](https://control.ros.org/master/doc/ros2_controllers/doc/writing_new_controller.html)
 - [UR GPIO Controller](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/blob/main/ur_controllers/include/ur_controllers/gpio_controller.hpp)
 - [SemanticComponentInterfaces example as used for the DashboardController](https://github.com/ICube-Robotics/iiwa_ros2/tree/main/iiwa_controllers/external_torque_sensor_broadcaster)
