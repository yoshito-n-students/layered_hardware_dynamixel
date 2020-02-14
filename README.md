# layered_hardware_dynamixel
A ros_control layer implementation for ROBOTIS Dynamixel actuators. See [layered_hardware](https://github.com/yoshito-n-students/layered_hardware) to understand the layered scheme.

## Plugins: layered_hardware_dynamixel_plugins
### layered_hardware_dynamixel/DynamixelActuatorLayer
* implements state & command interfaces for ROBOTIS Dynamixel actuators
#### <u>Layer parameters (should be defined under ~<layer_name>)</u>
___serial_interface___ (string, default: '/dev/ttyUSB0')
* path to Usb2Dynamixel device

___baudrate___ (int, default: 115200)
* baudrate for Usb2Dynamixel device

___actuators___ (struct, required)
* actuator parameters (see below)

#### <u>Actuator parameters (should be defined under ~<layer_name>/actuators/<actuator_name>)</u>
___id___ (int, required)
* id of the dynamixel actuator

___torque_constant___ (double, required)
* torque constant for conversion between current and torque in N*m/A
* ex. if the actuator's stall torque & current are 10.6 N*m & 4.4 A at the operating voltage, it would be 2.41 (= 10.6 / 4.4)

___operating_mode_map___ (map<string, string>, required)
* map from ROS's controller names to Dynamixel's operating mode names
* possible operating mode names are 'clear_multi_turn', 'current_based_position', 'current', 'extended_position', 'reboot', 'torque_disable', & 'velocity'

___item_map/<operating_mode_name>___ (map<string, int>, optional)
* pairs of Dynamixel's control table key & value
* the plugin will write values to the actuator just after changing operating modes
* ex. custom PID gains for the 'extended_position' mode
```
item_map:
  extended_position:
    Position_P_Gain: 800
    Position_I_Gain: 400
    Position_D_Gain: 0
```

#### <u>Example</u>
see [launch/single_dynamixel_example.launch](launch/single_dynamixel_example.launch)

## Tips
* if you feel slow communication speed with actuators, try adjusting the latency timer for your usb-serial device according to [this comment](https://github.com/ROBOTIS-GIT/DynamixelSDK/blob/3ae73bf5179fbad2bd366f39a952ce549c10c58e/c%2B%2B/src/dynamixel_sdk/port_handler_linux.cpp#L33-L56)