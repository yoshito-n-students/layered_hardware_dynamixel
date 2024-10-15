#include <layered_hardware/layer_interface.hpp>
#include <layered_hardware_dynamixel/dynamixel_actuator_layer.hpp>
#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(layered_hardware_dynamixel::DynamixelActuatorLayer,
                       layered_hardware::LayerInterface);