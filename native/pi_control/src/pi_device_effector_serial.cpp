/*!
 * @file pi_device_effector_serial.cpp
 * @brief Implementation of the DeviceEffectorSerial class for serial bus-servo effector device control.
 */

 #include <unistd.h>

#include "pi_device_effector_serial.hpp"
#include "pi_joint.hpp"
#include "pi_servo.hpp"

DeviceEffectorSerial::DeviceEffectorSerial(const CommandLineArgs& cla) : DeviceEffector(cla) {}

DeviceEffectorSerial::~DeviceEffectorSerial() {}

ReturnCode DeviceEffectorSerial::set_control_mode(Role target_role, ControlModeIntent intent) {
    ReturnCode rc = ReturnCode::SUCCESS;

    // READY_MOVE_OVERRIDE forces a safe position-based behavior regardless of configured effector control type.
    // See DeviceEffector::get_effective_control_mode() docs for why we use an override flag instead of mutating
    // the persistent control_mode_ (avoid save/restore and make restoration deterministic).
    if (intent == ControlModeIntent::READY_MOVE_OVERRIDE) {
        set_ready_move_force_position_mode(true);
    } else {
        set_ready_move_force_position_mode(false);
    }

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
    return ReturnCode::SUCCESS;
}

ReturnCode DeviceEffectorSerial::init(const CommandLineArgs& cla, int argc, char** argv, std::shared_ptr<Topic> p_topic,
                                      std::shared_ptr<Driver> p_driver) {
    ReturnCode return_code = DeviceEffector::init(cla, argc, argv, p_topic, p_driver);
    if (return_code != ReturnCode::SUCCESS) {
        PI_ERROR("Base effector initialization failed");
        return return_code;
    }

    ///< @note JSON configuration assumes effector servos follow arm servos in the data array. When no arm is
    ///< attached, effector servo data indices are adjusted to start from 0.
    if (p_arm_ == nullptr) {
        int servo_data_index = 0;
        for (auto& p_joint : joints_) {
            for (auto& p_servo : p_joint->get_servos()) {
                // data_index_ is a base Servo member; no family-specific cast
                // is needed (works for Dynamixel and FeeTech servos alike).
                p_servo->data_index_ = servo_data_index;
                servo_data_index++;
            }
        }
    }

    return return_code;
}

ReturnCode DeviceEffectorSerial::move_to_ready_position() {
    ReturnCode return_code = ReturnCode::SUCCESS;

    if (is_read_only() == true) {
        // Read-only serial effector (e.g. T6_2.0A leader-side or T6_2.0A as a follower
        // gripper that reports position but isn't actuated): mirror the base-class
        // short-circuit so the device's is_ready_ flag actually flips. Without this,
        // pi_control_node never publishes DEVICE_INFO_READY_NOW and the UI hangs in
        // "waiting for ready" forever.
        is_ready_ = true;
        PI_INFO("DeviceEffector", InfoLevel::ESSENTIAL_0,
                "%s_%s is read only, skipping move to ready position",
                model_.c_str(), id_.c_str());
        return ReturnCode::SUCCESS;
    }

    int i = 0;
    for (auto& p_joint : joints_) {
        tele_pos_[i] = p_joint->get_pos_rad_relative();

        // Engage torque once per ready move, not on every control cycle. Re-sending
        // enable(true) on the cycle where the effector reaches ready races the
        // leader-passive torque disable that follows in the same cycle: SO-ARM101
        // servos (Hiwonder HX firmware) ignore a torque-off arriving right after a
        // torque-on, leaving the leader gripper locked.
        if (ready_move_torque_engaged_ == false) {
            for (auto& p_servo : p_joint->get_servos()) {
                // enable_torque() is virtual on the Servo base class, so this path
                // works for every bus-servo family (Dynamixel, FeeTech).
                p_servo->enable_torque(true);
                usleep(100);
            }
        }

        i++;
    }
    ready_move_torque_engaged_ = true;

    return_code = DeviceEffector::move_to_ready_position();
    if (return_code != ReturnCode::SUCCESS) {
        PI_ERROR("Failed to move serial effector to ready position");
        return return_code;
    }

    if (is_ready_ == true) {
        // Control-mode restoration is handled by the generic move-to-ready command state machine
        // (Device::step()) via set_control_mode(...). Keep move_to_ready_position() focused on motion.

        PI_INFO("DeviceEffector", InfoLevel::DETAIL_2, "Ready flag set: %s_%s", model_.c_str(), id_.c_str());
    }

    return return_code;
}

ReturnCode DeviceEffectorSerial::move_joint_with_torque(Joint* p_joint, float target_pos) {
    ReturnCode return_code = ReturnCode::SUCCESS;

    if (is_read_only() == true) {
        // If the device is read only, skip the move joint with torque
        return ReturnCode::SUCCESS;
    }

    ///< @note Torque-based position control requires fast position monitoring (more than 100Hz) for stability

    float clipped_target_pos =
        p_joint->clipping(target_pos, p_joint->get_pos_min_relative(), p_joint->get_pos_max_relative());
    float clipped_curr_pos = p_joint->clipping(p_joint->get_pos_rad_relative(), p_joint->get_pos_min_relative(),
                                               p_joint->get_pos_max_relative());

    float distance = clipped_target_pos - clipped_curr_pos;

    float torq_to_apply = distance_to_torque_ * distance;

    ///< @note The torque is calculated from the parameter joint but applied to all joints
    for (auto& p_joint_iter : joints_) {
        return_code = p_joint_iter->move(clipped_target_pos, 0, torq_to_apply);
        if (return_code != ReturnCode::SUCCESS) {
            PI_ERROR("Failed to apply torque to joint %d in %s_%s", p_joint_iter->id_, model_.c_str(), id_.c_str());
            return return_code;
        }
    }

    PI_INFO("DeviceEffector", InfoLevel::FREQUENT_3,
            "Serial effector torque control: target_pos_rel=%.3f, clipped_target_pos=%.3f, curr_pos_rel=%.3f, "
            "clipped_curr_pos=%.3f, distance=%.3f, torq_to_apply=%.3f, distance_to_torque_=%.3f",
            target_pos, clipped_target_pos, p_joint->get_pos_rad_relative(), clipped_curr_pos, distance, torq_to_apply,
            distance_to_torque_);
    return return_code;
}
