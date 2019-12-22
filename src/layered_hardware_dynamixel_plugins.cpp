#include <layered_hardware/layer_base.hpp>
#include <layered_hardware_dynamixel/dynamixel_actuator_layer.hpp>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(layered_hardware_dynamixel::DynamixelActuatorLayer,
                       layered_hardware::LayerBase);