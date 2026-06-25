/*!
 * @file pi_device_arm_arx.cpp
 * @brief Implementation of the DeviceArmArx class for ARX robotic arm device control.
 */

#include <unistd.h>

#include "pi_device_arm_arx.hpp"

DeviceArmArx::DeviceArmArx(const CommandLineArgs& cla) : DeviceArm(cla) {}

DeviceArmArx::~DeviceArmArx() {}

ReturnCode DeviceArmArx::set_control_mode(Role target_role, ControlModeIntent intent) {
    // ARX family: control-mode switching is not required here.
    if (target_role == Role::FOLLOWER || intent == ControlModeIntent::READY_MOVE_OVERRIDE) {
        reset_slew_targets_to_current();
    }
    return ReturnCode::SUCCESS;
}
