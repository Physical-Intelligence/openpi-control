/*!
 * @file pi_device_arm_serial.cpp
 * @brief Implementation of the DeviceArmSerial class for serial bus-servo arm device control.
 */

#include <unistd.h>

#include "pi_device_arm_serial.hpp"
#include "pi_joint.hpp"
#include "pi_profile.hpp"

DeviceArmSerial::DeviceArmSerial(const CommandLineArgs& cla) : DeviceArm(cla) {}

DeviceArmSerial::~DeviceArmSerial() {}

ReturnCode DeviceArmSerial::set_control_mode(Role target_role, ControlModeIntent intent) {
    // Implementation note:
    // - Serial-arm leader/follower switching is already expressed via Joint::change_control_mode_for_{leader,follower}(),
    //   which maps to servo/driver operation modes.
    // - For READY_MOVE_OVERRIDE we still treat it as follower-like (position-based) so that move_to_ready_position()
    //   can safely send position targets from the current pose.
    // - After ready completion, normal operation will call this again with intent NORMAL_OPERATION and the actual role.
    ReturnCode rc = ReturnCode::SUCCESS;

    const bool use_follower_like = (intent == ControlModeIntent::READY_MOVE_OVERRIDE) || (target_role == Role::FOLLOWER);
    if (use_follower_like) {
        for (auto& p_joint : joints_) {
            rc = p_joint->change_control_mode_for_follower();
            if (rc != ReturnCode::SUCCESS) return rc;
        }
        return ReturnCode::SUCCESS;
    }

    // Leader
    prof_time_t current_time = Profile::get_time_now();
    for (auto& p_joint : joints_) {
        rc = p_joint->change_control_mode_for_leader(current_time);
        if (rc != ReturnCode::SUCCESS) return rc;
    }

    // Leader only: chain to the attached effector (same as DeviceArm::set_control_mode).
    // The effector's own step switches its mode once at its ready transition, but that single
    // attempt can fail -- the passive-leader torque disable races the final ready-move writes
    // on SO-ARM101 -- so re-applying here at arm-ready gives it a second, later chance. The
    // call is idempotent. The follower-like branch above intentionally does not chain: the serial arm
    // follower effector modes depend on the configured effector control type and are handled
    // by the effector's own ready transition, matching the long-standing serial-arm behavior.
    // Skipped during emergency recovery for the same bus-timeout reason as the base class.
    if (p_effector_ && !is_in_emergency_recovery()) {
        rc = p_effector_->set_control_mode(p_effector_->get_device_role(), intent);
        if (rc != ReturnCode::SUCCESS) return rc;
    }

    return ReturnCode::SUCCESS;
}

ReturnCode DeviceArmSerial::move_to_ready_position() {
    ReturnCode return_code = ReturnCode::SUCCESS;

    return_code = DeviceArm::move_to_ready_position();
    if (return_code != ReturnCode::SUCCESS) {
        PI_ERROR("Failed to move serial arm to ready position");
        return return_code;
    }

    if (is_ready_ == true) {
        // Control-mode restoration is handled by the generic move-to-ready command state machine
        // (Device::step()) via set_control_mode(...). Keep move_to_ready_position() focused on motion.

        PI_INFO("DeviceArm", InfoLevel::DETAIL_2, "Ready flag set: %s_%s", model_.c_str(), id_.c_str());
    }

    return return_code;
}
