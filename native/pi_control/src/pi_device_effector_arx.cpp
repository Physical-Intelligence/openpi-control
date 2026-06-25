/*!
 * @file pi_device_effector_arx.cpp
 * @brief Implementation of the DeviceEffectorArx class for ARX effector device control.
 */

#include <unistd.h>

#include "pi_device_effector_arx.hpp"
#include "pi_joint.hpp"

#define INIT_MOVE_TRY_MAX 200  ///< Maximum number of attempts to move effector to zero position

DeviceEffectorArx::DeviceEffectorArx(const CommandLineArgs& cla) : DeviceEffector(cla) {}

DeviceEffectorArx::~DeviceEffectorArx() {}

ReturnCode DeviceEffectorArx::set_control_mode(Role target_role, ControlModeIntent intent) {
    (void)target_role;
    (void)intent;
    ramped_target_initialized_ = false;
    // ARX family: control-mode switching is not required here.
    return ReturnCode::SUCCESS;
}

ReturnCode DeviceEffectorArx::move_joint_with_torque(Joint* p_joint, float target_pos) {
    ReturnCode return_code = ReturnCode::SUCCESS;

    // monopi ControlFollowGripper (control_follow.cc): the gripper is a
    // host-side torque spring,
    //     torque = clamp(constant * (goal - measured - offset), +/-bound),
    // sent as a torque-only command (kp=0; the servo's kd supplies damping).
    // A gripper blocked by a wide object saturates at the bound and squeezes
    // with that bounded force indefinitely instead of grinding at full motor
    // current (the i2rt failure mode: overheated gripper motors, snapped
    // fingers). The torque is linear and continuous through zero error --
    // saturation only flattens the tails -- so nothing sign-flips or chatters
    // near the target. Match monopi's non-L5 ARX command ramp: initialize at
    // the measured position and move the internal goal by at most 1 rad/tick.
    const float clipped_target_pos =
        p_joint->clipping(target_pos, p_joint->get_pos_min_relative(), p_joint->get_pos_max_relative());
    const float measured_pos = p_joint->get_pos_rad_relative();
    if (!ramped_target_initialized_) {
        ramped_target_pos_ = measured_pos;
        ramped_target_initialized_ = true;
    }
    const float target_delta = clipped_target_pos - ramped_target_pos_;
    ramped_target_pos_ += p_joint->clipping(target_delta, -1.0f, 1.0f);

    float torque = distance_to_torque_ * (ramped_target_pos_ - measured_pos - grip_spring_offset_);
    const float bound =
        (p_joint->grip_torque_limit_nm_ > 0.0f) ? p_joint->grip_torque_limit_nm_ : p_joint->torq_max_;
    torque = p_joint->clipping(torque, -bound, bound);

    // The error above lives in the joint-relative frame; torque commands reach
    // the motor raw, so a dir-inverted servo (E_Yam ships dir_invert -1) needs
    // the sign mapped back into the motor frame.
    torque *= p_joint->get_dir_invert();

    // monopi sends kp=0 with the gripper servo's configured kd=0.1. Keep that
    // damping local to this controller so arm gravity/torque frames retain
    // their existing zero-kd behavior.
    return_code = p_joint->apply_torque_with_damping(torque);

    if (return_code != ReturnCode::SUCCESS) {
        PI_ERROR("Failed to move joint %d with torque control in %s_%s", p_joint->id_, model_.c_str(), id_.c_str());
        return return_code;
    }

    return return_code;
}
