/*!
 * @file pi_device_arm_can.cpp
 * @brief Implementation of the DeviceArmCan class for MIT-mode CAN arm device control.
 */

#include <unistd.h>

#include "pi_device_arm_can.hpp"

DeviceArmCan::DeviceArmCan(const CommandLineArgs& cla) : DeviceArm(cla) {}

DeviceArmCan::~DeviceArmCan() {}

ReturnCode DeviceArmCan::set_control_mode(Role target_role, ControlModeIntent intent) {
    // MIT-mode CAN devices: control-mode switching is not required here.
    if (target_role == Role::FOLLOWER || intent == ControlModeIntent::READY_MOVE_OVERRIDE) {
        reset_slew_targets_to_current();
    }
    return ReturnCode::SUCCESS;
}
