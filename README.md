# layered_hardware_dynamixel
A ros2_control layer implementation for ROBOTIS Dynamixel actuators. See [layered_hardware](https://github.com/yoshito-n-students/layered_hardware/tree/humble) to understand the layered scheme.

## Layer plugin: layered_hardware_dynamixel/DynamixelActuatorLayer
* sends commands to ROBOTIS Dynamixel actuators within `write()` function
* fetches actuators' states within `read()` function
* switches actuators' operating modes within `perform_command_mode_swtich()` function when controllers using associated interfaces activate

### Hardware parameters
___<layer_name>___ (yaml, required)
* map of parameter names and values for this layer

___<layer_name>.serial_interface___ (string, default: '/dev/ttyUSB0')
* path to Usb2Dynamixel device

___<layer_name>.baudrate___ (int, default: 115200)
* baudrate for Usb2Dynamixel device

___<layer_name>.actuators___ (map<string, map>, required)
* map of parameters for each actuator

___<layer_name>.actuators.<actuator_name>.id___ (int, required)
* id of the dynamixel actuator

___<layer_name>.actuators.<actuator_name>.torque_constant___ (double, required)
* torque constant for conversion between current and torque in N*m/A
* ex. if the actuator's stall torque & current are 10.6 N*m & 4.4 A at the operating voltage, it would be 2.41 (= 10.6 / 4.4)

___<layer_name>.actuators.<actuator_name>.operating_mode_map___ (map<string, string>, required)
* map to actuator's operating mode names from associated interface names (typically joint interfaces)
* possible operating mode names are 'clear_multi_turn', 'current_based_position', 'current', 'extended_position', 'reboot', 'torque_disable', & 'velocity'

### Example of parameter description
```yaml
<param name="example_dynamixel_actuator_layer">
    serial_interface: /dev/serial/by-id/...
    baudrate: 1000000
    actuators:
        example_dynamixel_1:
            id: 1
            torque_constant: 2.41
            operating_mode_map:
                example_joint_1/position: extended_position
                ...
        example_dynamixel_2:
            id: 2
            ...
</param>
```

## Example
* [single Dynamixel actuator](examples/single_dynamixel)

## Tips
* if you feel slow communication speed with actuators, try adjusting the latency timer for your usb-serial device according to [this comment](https://github.com/ROBOTIS-GIT/DynamixelSDK/blob/3ae73bf5179fbad2bd366f39a952ce549c10c58e/c%2B%2B/src/dynamixel_sdk/port_handler_linux.cpp#L33-L56)