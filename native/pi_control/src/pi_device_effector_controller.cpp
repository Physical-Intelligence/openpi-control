/*!
 * @file pi_device_effector_controller.cpp
 * @brief Implementation of the DeviceEffectorController class.
 */

#include "pi_device_effector_controller.hpp"

DeviceEffectorController::DeviceEffectorController(const CommandLineArgs& cla) : DeviceEffectorCan(cla) {}

DeviceEffectorController::~DeviceEffectorController() {}

ReturnCode DeviceEffectorController::set_control_mode(Role target_role, ControlModeIntent intent) {
    // Controller-managed grippers switch between position (follower / ready
    // move) and external effort (leader / torque gripper) on the vendor
    // controller; the DeviceEffector base implementation delegates exactly
    // that to the joints.
    return DeviceEffector::set_control_mode(target_role, intent);
}
